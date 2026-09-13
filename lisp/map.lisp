; Signed-byte cells stay in RAM; code and immutable constants live in flash.
;
; The grid is deliberately ragged, because a rectangular one would spend a
; quarter of the EEPROM on a region with no content: negative duty under a
; positive throttle only ever means "full forward torque".
;
;   rows 0..9   brake levers -100%..-10%, 10% steps, 21 duty columns
;               spanning -100%..+100% - braking against speed on the right,
;               reverse on the left
;   rows 10..30 throttle 0..100%, 5% steps, 11 duty columns spanning
;               0..100%; negative duty reads as column 0
;
; 10*21 + 21*11 = 441 cells, the same budget a 21x21 grid would have used.
@const-start
(define map-thr-n 31)
(define map-duty-n 21)
(define map-thr-zero 10)
(define map-brake-cells 210)
(define map-cells 441)
@const-end
(define map-buf (array-create map-cells))
@const-start
; Columns a given row actually stores.
(defun map-row-cols (t-i) (if (< t-i 10) 21 11))
(defun map-idx (t-i d-i)
    (if (< t-i 10)
        (+ (* t-i 21) d-i)
        (+ 210 (* (- t-i 10) 11) d-i)))
(defun map-get-cell (t-i d-i) (* (bufget-i8 map-buf (map-idx t-i d-i)) 10))
(defun map-set-cell (t-i d-i val)
    (bufset-i8 map-buf (map-idx t-i d-i) (cell-to-i8 val)))

; Zero lever is not stored: it is zero by definition, and having it lets a
; light pull fade in from nothing instead of jumping to the -10% row.
(defun map-brake-get (p d-i) (if (= p 0) 0 (map-get-cell (- 10 p) d-i)))

; ---- lookup --------------------------------------------------------------
; Both halves interpolate on their own uniform grid; nothing ever needs to
; blend across the seam, since throttle and brake are separate inputs.
; All intermediates fit the VESC's signed 28-bit inline integer.

(defun map-lookup-brake (lev duty)
    (let ((lf (* (clamp-f lev 0 1000) 10))
          (df (* (+ (clamp-f duty -1000 1000) 1000) 10))
          (p0 (min-f (/ lf 1000) 10)) (d0 (min-f (/ df 1000) 20))
          (p1 (min-f (+ p0 1) 10)) (d1 (min-f (+ d0 1) 20))
          (pw (mod lf 1000)) (dw (mod df 1000))
          (v00 (map-brake-get p0 d0)) (v01 (map-brake-get p0 d1))
          (v10 (map-brake-get p1 d0)) (v11 (map-brake-get p1 d1))
          (v0 (+ v00 (/ (* (- v01 v00) dw) 1000)))
          (v1 (+ v10 (/ (* (- v11 v10) dw) 1000))))
        (+ v0 (/ (* (- v1 v0) pw) 1000))))

(defun map-lookup-drive (thr duty)
    (let ((tf (* (clamp-f thr 0 1000) 20))
          (df (* (clamp-f duty 0 1000) 10))
          (t0 (min-f (+ 10 (/ tf 1000)) 30)) (d0 (min-f (/ df 1000) 10))
          (t1 (min-f (+ t0 1) 30)) (d1 (min-f (+ d0 1) 10))
          (tw (mod tf 1000)) (dw (mod df 1000))
          (v00 (map-get-cell t0 d0)) (v01 (map-get-cell t0 d1))
          (v10 (map-get-cell t1 d0)) (v11 (map-get-cell t1 d1))
          (v0 (+ v00 (/ (* (- v01 v00) dw) 1000)))
          (v1 (+ v10 (/ (* (- v11 v10) dw) 1000))))
        (+ v0 (/ (* (- v1 v0) tw) 1000))))

(defun map-lookup (thr duty)
    (if (< thr 0)
        (map-lookup-brake (- thr) duty)
        (map-lookup-drive thr duty)))

; ---- generators ----------------------------------------------------------

(defun thermal-peak (thr response hold)
    (let ((base (pow thr response)))
        (+ base (* (- thr base) hold (clamp01 (/ (- thr 0.6) 0.4))))))

(defun shape-curve (p shape)
    (cond ((= shape 0) p)
          ((= shape 1) (* p p))
          ((= shape 2) (- 1.0 (pow (- 1.0 p) 3)))
          (t (pow p 4))))

(defun thermal-cell (thr duty peak balance coupling width shape brake overrun curve)
    (cond
        ((< thr 0.02) (- (* brake (pow duty (+ 1.0 curve)))))
        ; Zero speed coupling selects duty-independent electric torque.
        ((= coupling 0.0) peak)
        (t
            (let ((start (clamp01 (- balance width))))
                (cond
                    ((<= duty start) peak)
                    ((<= duty balance)
                        (* peak (- 1.0 (shape-curve
                            (/ (- duty start) (max-f 0.001 (- balance start))) shape))))
                    (t (- (* overrun (pow
                        (clamp01 (/ (- duty balance) (max-f 0.001 (- 1.0 balance))))
                        (+ 1.0 curve))))))))))

(defun gen-thermal-map (response coupling width shape hold brake overrun curve)
    (looprange ti 0 21
        (let ((thr (/ ti 20.0))
              (peak (thermal-peak thr response hold))
              (balance (clamp01 (* thr coupling))))
            (looprange di 0 11
                (map-set-cell (+ ti 10) di
                    (to-fp (thermal-cell thr (/ di 10.0) peak balance coupling
                                        width shape brake overrun curve)))))))

; Brake rows. Forward duty carries braking against speed; negative duty
; carries reverse, with the traction law's shape mirrored onto it.
;   peak    = strength * lever ^ response
;   forward : -(peak * ((1 - dep) + dep * duty ^ (1 + curve)))
;   reverse : balance = clamp(lever * rev_coupling); below it -peak,
;             tapering to zero at balance, then positive - forward torque,
;             which holds a reverse that is running away.
(defun brake-cell (b duty strength resp dep curve rcoup rwidth rover)
    (let ((peak (* strength (pow b resp))))
        (if (> duty 0.0)
            (- (* peak (clamp01 (+ (- 1.0 dep) (* dep (pow duty (+ 1.0 curve)))))))
            (if (= rcoup 0.0)
                (- peak)
                (let ((rev (- duty))
                      (balance (clamp01 (* b rcoup))))
                    (let ((start (clamp01 (- balance rwidth))))
                        (cond
                            ((<= rev start) (- peak))
                            ((<= rev balance)
                                (- (* peak (- 1.0 (shape-curve
                                    (/ (- rev start) (max-f 0.001 (- balance start))) 1)))))
                            (t (* rover (pow
                                (clamp01 (/ (- rev balance) (max-f 0.001 (- 1.0 balance))))
                                2.0))))))))))

(defun gen-brake-map (strength resp dep curve rcoup rwidth rover)
    (looprange bi 0 10
        (let ((b (/ (- 10 bi) 10.0)))
            (looprange di 0 21
                (map-set-cell bi di
                    (to-fp (brake-cell b (/ (- di 10) 10.0)
                                       strength resp dep curve rcoup rwidth rover)))))))
@const-end
