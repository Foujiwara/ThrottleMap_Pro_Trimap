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

; Brake half generator: braking demand against FORWARD speed, on the same
; |duty| axis as the traction half, so the whole map reads the same way.
;   peak  = strength * lever ^ response
;   speed = (1 - duty_dep) + duty_dep * duty ^ (1 + curve)
;   cell  = -(peak * speed)
; duty_dep 0.0 (the default) is flat: -10% lever is -10% current at any
; speed. Raise it to fade braking out at low speed, or invert the feel
; with the curve. How far back the lever may drive is NOT in these cells -
; see brake-reverse in package.lisp.
(defun brake-cell (b duty strength resp dep curve)
    (let ((peak (* strength (pow b resp)))
          (speed (clamp01 (+ (- 1.0 dep) (* dep (pow duty (+ 1.0 curve)))))))
        (- (* peak speed))))

(defun gen-brake-map (strength resp dep curve)
    (looprange bi 0 10
        (let ((b (/ (- 10 bi) 10.0)))
            (looprange di 0 11
                (map-set-cell bi di
                    (to-fp (brake-cell b (/ di 10.0) strength resp dep curve)))))))
@const-end
