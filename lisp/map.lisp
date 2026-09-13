; Signed-byte cells stay in RAM; code and immutable constants live in flash.
; Vertical axis spans -100%..+100% throttle: rows 0..9 are the brake half
; (10% steps), row 10 is zero, rows 10..30 are traction (5% steps).
@const-start
(define map-thr-n 31)
(define map-duty-n 11)
(define map-thr-zero 10)
(define map-cells 341)
@const-end
(define map-buf (array-create map-cells))
@const-start
(defun map-idx (t-i d-i) (+ (* t-i 11) d-i))
(defun map-get-cell (t-i d-i) (* (bufget-i8 map-buf (map-idx t-i d-i)) 10))
(defun map-set-cell (t-i d-i val)
    (bufset-i8 map-buf (map-idx t-i d-i) (cell-to-i8 val)))

; Row position of a signed throttle, x1000. The two halves have different
; steps, so the scale depends on the sign; the +10000 bias keeps the value
; positive so `mod` behaves. All intermediates fit a 28-bit inline integer.
(defun map-row-pos (thr)
    (let ((tc (clamp-f thr -1000 1000)))
        (+ 10000 (* tc (if (< tc 0) 10 20)))))

(defun map-lookup (thr duty)
    (let ((tf (map-row-pos thr))
          (df (* (clamp-f duty 0 1000) 10))
          (t0 (min-f (/ tf 1000) 30)) (d0 (/ df 1000))
          (t1 (min-f (+ t0 1) 30)) (d1 (min-f (+ d0 1) 10))
          (tw (mod tf 1000)) (dw (mod df 1000))
          (v00 (map-get-cell t0 d0)) (v01 (map-get-cell t0 d1))
          (v10 (map-get-cell t1 d0)) (v11 (map-get-cell t1 d1))
          (v0 (+ v00 (/ (* (- v01 v00) dw) 1000)))
          (v1 (+ v10 (/ (* (- v11 v10) dw) 1000))))
        (+ v0 (/ (* (- v1 v0) tw) 1000))))

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

; Traction half only: the brake rows are hand territory and no slider ever
; rewrites them.
(defun gen-thermal-map (response coupling width shape hold brake overrun curve)
    (looprange ti 0 21
        (let ((thr (/ ti 20.0))
              (peak (thermal-peak thr response hold))
              (balance (clamp01 (* thr coupling))))
            (looprange di 0 11
                (map-set-cell (+ ti 10) di
                    (to-fp (thermal-cell thr (/ di 10.0) peak balance coupling
                                        width shape brake overrun curve)))))))

; Brake half generator - the mirror of thermal-cell. Its duty axis is
; REVERSE duty, so column 0 is "not going backwards at all" and holds the
; full demand for that lever position: -10% lever is -10% current while
; travelling forwards, at any speed.
;   peak    = strength * lever ^ response
;   balance = clamp(lever * coupling)     reverse duty it settles at
; Below balance it pulls backwards at peak, tapers to zero over width,
; and past balance turns positive - forward torque, which decelerates a
; reverse that has run away. coupling 0 removes the balance entirely and
; leaves a plain plateau.
(defun brake-cell (b rev strength resp coupling width overrun curve)
    (let ((peak (* strength (pow b resp))))
        (if (= coupling 0.0)
            (- peak)
            (let ((balance (clamp01 (* b coupling)))
                  (start (clamp01 (- (clamp01 (* b coupling)) width))))
                (cond
                    ((<= rev start) (- peak))
                    ((<= rev balance)
                        (- (* peak (- 1.0 (shape-curve
                            (/ (- rev start) (max-f 0.001 (- balance start))) 1)))))
                    (t (* overrun (pow
                        (clamp01 (/ (- rev balance) (max-f 0.001 (- 1.0 balance))))
                        (+ 1.0 curve)))))))))

(defun gen-brake-map (strength resp coupling width overrun curve)
    (looprange bi 0 10
        (let ((b (/ (- 10 bi) 10.0)))
            (looprange di 0 11
                (map-set-cell bi di
                    (to-fp (brake-cell b (/ di 10.0) strength resp coupling
                                       width overrun curve)))))))
@const-end
