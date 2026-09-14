; Input values and filters use inline 28-bit fixed-point integers.
(define thr-cfg-source 0)
(define thr-cfg-invert 0)
(define thr-cfg-deadband 20)
(define thr-cfg-filter 1000)
(define thr-cfg-brake-mode 0)
; Filter accumulator has an extra x1000 precision to avoid sticking above
; zero when the integer EMA delta rounds down.
(define thr-filtered 0)
(define thr-expired nil)
(define thr-filter-acc 0)
(define thr-adc-cal-start 0)
(define thr-adc-cal-end 0)
(define thr-adc-cal-loaded nil)
(define thr-bidir-center 1650)
; Set when the bidirectional input reads outside its calibrated band. A
; centre-resting input has no safe failure value: a broken signal wire sits
; near 0 V, which is a full reverse request and is indistinguishable from a
; deliberate one without this test.
(define thr-adc-fault nil)
; Brake channel filter state. The throttle has always been smoothed; the
; dual-ADC brake was raw ADC straight into the map.
(define thr-brake-acc 0)
(define thr-brake-filtered 0)
(define uart-buf (array-create 3))
(define uart-started nil)
(define uart-pos 0)
(define uart-percent 0)
(define uart-last-raw 0)
(define uart-last-time 0)
(define thr-test-value 0)
(define thr-test-time 0)

@const-start
(define thr-src-adc 0)
(define thr-src-ppm 1)
(define thr-src-uart 2)
(define thr-src-test 3)
(define thr-brake-none 0)
(define thr-brake-dual 1)
(define thr-brake-bidir 2)

(defun thr-reset-state ()
    (progn
        (setq thr-filtered 0)
        (setq thr-filter-acc 0)
        (setq thr-test-value 0)
        (setq uart-last-raw 0)
        (setq uart-pos 0)
        (setq thr-brake-acc 0)
        (setq thr-brake-filtered 0)
        (setq thr-adc-fault nil)
        (setq thr-adc-cal-loaded nil)))

(defun thr-adc-cal-ensure ()
    (if (not thr-adc-cal-loaded)
        (progn
            (setq thr-adc-cal-start (to-fp (conf-get 'adc-v1-start)))
            (setq thr-adc-cal-end (to-fp (conf-get 'adc-v1-end)))
            (setq thr-adc-cal-loaded t))))

; One voltage sample per control tick, shared with the brake path.
; A reversed ADC calibration works too; equal endpoints produce zero.
(defun thr-adc-signed ()
    (progn
        (thr-adc-cal-ensure)
        (let ((dir (if (> thr-adc-cal-end thr-adc-cal-start) 1 -1))
              (raw (to-fp (get-adc 0)))
              (lo (min-f thr-adc-cal-start thr-adc-cal-end))
              (hi (max-f thr-adc-cal-start thr-adc-cal-end)))
            (progn
            ; Only meaningful once the band is plausibly calibrated: a start
            ; at 0 V cannot be told apart from a dead wire.
            (setq thr-adc-fault
                (and (> lo 150)
                     (or (< raw (- lo 150)) (> raw (+ hi 150)))))
            (if thr-adc-fault 0
            (let ((delta (* (- raw thr-bidir-center) dir))
                  (pos-span (* (- thr-adc-cal-end thr-bidir-center) dir))
                  (neg-span (* (- thr-bidir-center thr-adc-cal-start) dir)))
                (if (or (<= pos-span 0) (<= neg-span 0)) 0
                (let ((v (if (>= delta 0)
                             (clamp-f (/ (* delta 1000) pos-span) 0 1000)
                             (clamp-f (/ (* delta 1000) neg-span) -1000 0)))
                      (mag (abs v)))
                    (if (<= mag thr-cfg-deadband) 0
                        (* (if (< v 0) -1 1)
                           (/ (* (- mag thr-cfg-deadband) 1000)
                              (- 1000 thr-cfg-deadband))))))))))))

; Captures only the neutral point of the bidirectional input. Start/end stay
; owned by VESC Tool's ADC calibration.
(defun thr-calibrate-bidir-center ()
    (setq thr-bidir-center (clamp-f (to-fp (get-adc 0)) 0 3300)))

; Incremental parser preserves partial frames and resynchronizes after noise.
(defun uart-throttle-byte (v)
    (cond
        ((= uart-pos 0) (if (= v 0xA5) (setq uart-pos 1)))
        ((= uart-pos 1)
            (if (<= v 200)
                (progn (setq uart-percent v) (setq uart-pos 2))
                (setq uart-pos 0)))
        (t
            (if (= v (bitwise-xor 0xA5 uart-percent))
                (progn
                    (setq uart-last-raw (* uart-percent 5))
                    (setq uart-last-time (systime))
                    (setq uart-pos 0))
                (setq uart-pos (if (= v 0xA5) 1 0))))))

(defun uart-throttle-raw ()
    (progn
        (if (not uart-started)
            (progn (uart-start 115200) (setq uart-started t)))
        (let ((n (uart-read uart-buf 3 0 nil 0.0)))
            (looprange i 0 n (uart-throttle-byte (bufget-u8 uart-buf i))))
        uart-last-raw))

(defun thr-read-raw ()
    (cond
        ((= thr-cfg-source thr-src-adc)
            ; Signed on purpose: thr-read filters this value, and
            ; thr-brake-read takes the brake side off the same filtered
            ; result. Clipping here would leave the brake channel dead.
            (if (= thr-cfg-brake-mode thr-brake-bidir)
                (thr-adc-signed)
                (to-fp (get-adc-decoded 0))))
        ((= thr-cfg-source thr-src-ppm) (max-f 0 (to-fp (get-ppm))))
        ((= thr-cfg-source thr-src-uart) (uart-throttle-raw))
        ((= thr-cfg-source thr-src-test) thr-test-value)
        (t 0)))

(defun thr-deadband (v)
    (if (<= v thr-cfg-deadband) 0
        (/ (* (- v thr-cfg-deadband) 1000) (- 1000 thr-cfg-deadband))))

; PPM and UART only; ADC defers to VESC Tool's own Start/End calibration.
(defun thr-normalize (raw)
    (let ((v (clamp-f (/ (* (- raw 20) 1000) 960) 0 1000)))
        ; Inversion must precede deadband, or inverted rest commands torque.
        (thr-deadband (if (= thr-cfg-invert 1) (- 1000 v) v))))

(defun thr-input-expired ()
    (or (and (= thr-cfg-source thr-src-adc)
             (= thr-cfg-brake-mode thr-brake-bidir)
             thr-adc-fault)
        (and (= thr-cfg-source thr-src-test) (> (secs-since thr-test-time) 0.5))
        (and (= thr-cfg-source thr-src-ppm) (> (get-ppm-age) 0.5))
        (and (= thr-cfg-source thr-src-uart) (> (secs-since uart-last-time) 0.5))))

(defun thr-read ()
    (let ((raw (thr-read-raw))
          (n (if (= thr-cfg-source thr-src-adc)
                 (if (= thr-cfg-brake-mode thr-brake-bidir) raw (thr-deadband raw))
                 (if (= thr-cfg-source thr-src-test) (clamp-f raw 0 1000) (thr-normalize raw)))))
        (progn (setq thr-expired (thr-input-expired))
        (if thr-expired
            (progn (setq thr-filter-acc 0) (setq thr-filtered 0))
            (progn
                ; Split product to stay below the 28-bit signed limit.
                (setq thr-filter-acc
                    (+ thr-filter-acc
                       (* thr-cfg-filter (- n (/ thr-filter-acc 1000)))
                       (- (/ (* thr-cfg-filter (mod thr-filter-acc 1000)) 1000))))
                (setq thr-filtered (/ thr-filter-acc 1000))))
        (max-f 0 thr-filtered))))

; The dual-ADC brake gets the same smoothing as the throttle. It used to go
; from raw ADC straight into the map, so the two channels of the same pair
; of hall grips behaved completely differently: the throttle was filtered
; and the brake passed its noise on to the current request unchanged.
; Called exactly once per control tick, like thr-read.
(defun thr-brake-filter (n)
    (if thr-expired
        (progn (setq thr-brake-acc 0) (setq thr-brake-filtered 0) 0)
        (progn
            ; Split product to stay below the 28-bit signed limit.
            (setq thr-brake-acc
                (+ thr-brake-acc
                   (* thr-cfg-filter (- n (/ thr-brake-acc 1000)))
                   (- (/ (* thr-cfg-filter (mod thr-brake-acc 1000)) 1000))))
            (setq thr-brake-filtered (/ thr-brake-acc 1000))
            (max-f 0 thr-brake-filtered))))

(defun thr-brake-read ()
    (cond
        ; Bench source: a negative test value is a brake request.
        ((= thr-cfg-source thr-src-test) (max-f 0 (- thr-test-value)))
        ((not (= thr-cfg-source thr-src-adc)) 0)
        ((= thr-cfg-brake-mode thr-brake-dual)
            (thr-brake-filter (thr-deadband (to-fp (get-adc-decoded 1)))))
        ; Bidirectional takes the brake side off the already filtered signed
        ; sample, so it needs no second filter.
        ((= thr-cfg-brake-mode thr-brake-bidir) (max-f 0 (- thr-filtered)))
        (t 0)))
@const-end
