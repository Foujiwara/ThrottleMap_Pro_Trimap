; Signed-byte cells stay in RAM; code and immutable constants live in flash.
;
; The grid is deliberately ragged, because a rectangular one would spend a
; quarter of the EEPROM on a region that holds no independent information:
; for traction, duty is speed and its sign does not matter, so the negative
; side is the positive side mirrored.
;
;   rows 0..10  brake levers -100%..0, 10% steps, 21 duty columns spanning
;               -100%..+100% - braking against speed on the right, reverse
;               on the left. Row 10 is the released row, so its left half is
;               engine braking while rolling backwards, editable on its own.
;   rows 11..30 throttle 5..100%, 5% steps, 11 duty columns spanning
;               0..100%, read mirrored for negative duty
;
; 11*21 + 20*11 = 451 cells. That fills the EEPROM exactly: 126 of the 127
; data slots, with slot 127 holding the CRC.
@const-start
(define map-thr-n 31)
(define map-duty-n 21)
(define map-thr-zero 10)
(define map-cells 451)
@const-end
(define map-buf (array-create map-cells))
@const-start
; Columns a given row actually stores.
(defun map-row-cols (t-i) (if (<= t-i 10) 21 11))
(defun map-idx (t-i d-i)
    (if (<= t-i 10)
        (+ (* t-i 21) d-i)
        (+ 231 (* (- t-i 11) 11) d-i)))
(defun map-get-cell (t-i d-i) (* (bufget-i8 map-buf (map-idx t-i d-i)) 10))
(defun map-set-cell (t-i d-i val)
    (bufset-i8 map-buf (map-idx t-i d-i) (cell-to-i8 val)))

; Zero lever is row 10, the released row, so a light pull fades in from
; whatever engine braking is doing rather than from nothing.
(defun map-brake-get (p d-i) (map-get-cell (- 10 p) d-i))
; Row 10 stores the full 21 columns, so forward duty sits at 10 + d there.
(defun map-drive-get (t-i d-i)
    (map-get-cell t-i (if (= t-i 10) (+ d-i 10) d-i)))

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

; Mirrored on duty: rolling backwards reads the same curve as rolling
; forwards at that speed, so engine braking and the balance point work in
; both directions, with no discontinuity through zero.
(defun map-lookup-drive (thr duty)
    (let ((tf (* (clamp-f thr 0 1000) 20))
          (df (* (clamp-f (abs duty) 0 1000) 10))
          (t0 (min-f (+ 10 (/ tf 1000)) 30)) (d0 (min-f (/ df 1000) 10))
          (t1 (min-f (+ t0 1) 30)) (d1 (min-f (+ d0 1) 10))
          (tw (mod tf 1000)) (dw (mod df 1000))
          (v00 (map-drive-get t0 d0)) (v01 (map-drive-get t0 d1))
          (v10 (map-drive-get t1 d0)) (v11 (map-drive-get t1 d1))
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
    (progn
        (looprange ti 0 21
            (let ((thr (/ ti 20.0))
                  (peak (thermal-peak thr response hold))
                  (balance (clamp01 (* thr coupling))))
                (looprange di 0 11
                    (map-set-cell (+ ti 10) (if (= ti 0) (+ di 10) di)
                        (to-fp (thermal-cell thr (/ di 10.0) peak balance coupling
                                            width shape brake overrun curve))))))
        ; Released row, negative duty: engine braking while rolling
        ; backwards. Seeded as the mirror, editable on its own from there.
        (looprange di 0 10
            (map-set-cell 10 di
                (to-fp (thermal-cell 0.0 (/ (- 10 di) 10.0) 0.0 0.0 coupling
                                    width shape brake overrun curve))))))

; Brake rows split into their two halves, each with its own regenerate
; flag so shaping one never discards hand edits made to the other. Both
; read the cfg-* globals directly rather than taking a dozen arguments.

; Right half (duty > 0): braking against forward speed.
;   -(strength * lever^response * ((1 - dep) + dep * duty^(1 + curve)))
(defun brake-speed (duty)
    (clamp01 (+ (- 1.0 cfg-brake-dep)
                (* cfg-brake-dep (pow duty (+ 1.0 cfg-brake-curve))))))

(defun gen-brake-half ()
    (looprange bi 0 10
        (let ((peak (* cfg-brake-str (pow (/ (- 10 bi) 10.0) cfg-brake-resp))))
            (looprange di 11 21
                (map-set-cell bi di
                    (to-fp (- (* peak (brake-speed (/ (- di 10) 10.0))))))))))

; Left half (duty <= 0): reverse, which is the traction law negated - same
; peak/balance/taper/overrun shape, its own eight settings.
(defun gen-rev-half ()
    (looprange bi 0 10
        (let ((b (/ (- 10 bi) 10.0)))
            (let ((peak (* cfg-rev-str (thermal-peak b cfg-rev-resp cfg-rev-hold)))
                  (balance (clamp01 (* b cfg-rev-coupling))))
                (looprange di 0 11
                    (let ((rev (/ (- 10 di) 10.0)))
                        (map-set-cell bi di
                            (to-fp (- (thermal-cell b rev peak balance
                                        cfg-rev-coupling cfg-rev-width cfg-rev-shape
                                        0.0 cfg-rev-overrun cfg-rev-curve))))))))))
@const-end
