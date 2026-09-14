; Package entry point. Imports are read-only source; mutable data is RAM.
(import "util.lisp" 'bin-util)
(import "map.lisp" 'bin-map)
(import "throttle.lisp" 'bin-throttle)
(import "storage.lisp" 'bin-storage)
(import "protocol.lisp" 'bin-protocol)
(read-eval-program bin-util)
(read-eval-program bin-map)
(read-eval-program bin-throttle)
(read-eval-program bin-storage)
(read-eval-program bin-protocol)

(define cfg-preset 1)
(define cfg-torque-resp 0.85)
(define cfg-speed-coupling 1.0)
(define cfg-trans-width 0.10)
(define cfg-trans-shape 1)
(define cfg-high-hold 0.55)
(define cfg-engine-brake 0.15)
(define cfg-overrun-regen 0.12)
(define cfg-regen-curve 1)
; Brake half of the map: off keeps the plain proportional lever.
(define cfg-brake-map 0)
; 0 regen only, 1 current no reverse, 2 current bidirectional.
(define cfg-brake-type 0)
; Wire byte 19 / echo byte 29 are reserved: the ERPM switch threshold they
; used to carry is gone, replaced by the direction latch below.
; Brake half generator - see gen-brake-map in map.lisp.
(define cfg-brake-str 1.0)
(define cfg-brake-resp 1.0)
(define cfg-brake-dep 0.0)
(define cfg-brake-curve 0)
; Reverse generator - the traction law's eight settings, mirrored onto the
; brake rows' negative-duty columns.
(define cfg-rev-str 1.0)
(define cfg-rev-resp 1.0)
(define cfg-rev-hold 0.0)
(define cfg-rev-coupling 1.0)
(define cfg-rev-width 0.10)
(define cfg-rev-shape 1)
(define cfg-rev-overrun 0.12)
(define cfg-rev-curve 1)
; Duty is the variable the map closes its loop on, so it needs filtering at
; least as much as the throttle does. 1000 = unfiltered.
(define cfg-duty-filter 300)
; Milliseconds for the output to travel full scale. 0 = no limit.
(define cfg-slew-ms 150)
(define duty-filt-acc 0)
(define live-cmd 0)
(define live-throttle 0)
(define live-duty 0)
(define live-cur-rel 0)
(define live-brake 0)
; Direction latch for the no-reverse brake type. A bare (> rpm 0) test
; oscillates: the negative current it gates is exactly what drives the rpm
; through zero, so the decision flips every tick. Two thresholds instead of
; one - enter below 50 ERPM, leave only above 300 - and nothing in between
; can change the state.
(define rev-blocked nil)
; One buffer per owner: telemetry never shares its buffer with the event task.
(define row-packet (array-create 44))
(define cfg-packet (array-create 52))

@const-start
(defun rev-guard-update ()
    (let ((r (to-i (get-rpm))))
        (progn
            (if (> r 300) (setq rev-blocked nil))
            (if (< r 50) (setq rev-blocked t)))))

; How a negative output is commanded is the brake type's job alone, and it
; applies in every input mode - engine braking on a released throttle
; included, which is the whole point of offering the choice in Normal mode
; where there is no brake channel at all.
;   0 regen only   - set-brake-rel, which can only ever slow the motor
;   1 no reverse   - negative current down to a stop, then zero
;   2 bidirectional- signed current throughout, on into reverse
(defun apply-output (v)
    (if (>= v 0)
        (set-current-rel (fp-to-f v))
        (if (= cfg-brake-type 0)
            (set-brake-rel (fp-to-f (- v)))
            (progn
                (rev-guard-update)
                (if (and (= cfg-brake-type 1) rev-blocked)
                    (set-current-rel 0.0)
                    (set-current-rel (fp-to-f v)))))))

; Filtered duty. Same split-product EMA as the throttle: the accumulator
; carries an extra x1000 so it can actually reach its target instead of
; stalling a rounding step short. Signed, and the truncating division and
; mod agree on sign, so negative duty filters correctly too.
(defun duty-read ()
    (let ((d (to-fp (get-duty))))
        (if (>= cfg-duty-filter 1000)
            d
            (progn
                (setq duty-filt-acc
                    (+ duty-filt-acc
                       (* cfg-duty-filter (- d (/ duty-filt-acc 1000)))
                       (- (/ (* cfg-duty-filter (mod duty-filt-acc 1000)) 1000))))
                (/ duty-filt-acc 1000)))))

; Rate limit on the command itself. The VESC's own apps ramp their output;
; ours bypasses that entirely because their control type is Off, so without
; this every step in the map leaves as an instant torque step.
(defun slew-step (cur target)
    (if (= cfg-slew-ms 0)
        target
        (let ((step (max-f 1 (/ 5000 cfg-slew-ms))))
            (if (> target (+ cur step))
                (+ cur step)
                (if (< target (- cur step)) (- cur step) target)))))

(defun control-tick ()
    (if storage-busy
        (progn (setq live-cur-rel 0) (setq live-cmd 0) (setq duty-filt-acc 0))
        (progn
            (setq live-throttle (thr-read))
            (setq live-brake (thr-brake-read))
            (setq live-duty (duty-read))
            (let ((lever (> live-brake 0)))
                (progn
                    (setq live-cur-rel
                        (if thr-expired 0
                            (if lever
                                (if (= cfg-brake-map 0)
                                    (- live-brake)
                                    ; Positive cells in the brake rows are
                                    ; forward torque, which holds a runaway
                                    ; reverse. Only a bidirectional brake can
                                    ; get there, so the other types never see
                                    ; anything but braking.
                                    (let ((v (map-lookup (- live-brake) live-duty)))
                                        (if (= cfg-brake-type 2) v (min-f v 0))))
                                (map-lookup live-throttle live-duty))))
                    ; A lost input must cut instantly, never ramp down.
                    (setq live-cmd
                        (if thr-expired 0 (slew-step live-cmd live-cur-rel)))
                    (apply-output live-cmd))))))

(defun control-loop ()
    (loopwhile t (progn (control-tick) (sleep 0.005))))

(defun telemetry-loop ()
    (let ((b (array-create 17)))
        (progn (bufset-u8 b 0 pkt-live)
        (loopwhile t
            (progn
                (bufset-i16 b 1 live-throttle)
                (bufset-i16 b 3 live-duty)
                ; RPM is UI-only: sample at 20 Hz, not 200 Hz.
                (bufset-i32 b 5 (to-i (get-rpm)))
                (bufset-i16 b 9 live-cur-rel)
                (bufset-i16 b 11 (clamp-f (to-i (* (get-current) 100.0)) -32768 32767))
                (bufset-i16 b 13 live-brake)
                ; ADC1 voltage in millivolts for bidirectional calibration.
                (bufset-i16 b 15 (clamp-f (to-i (* (get-adc 0) 1000.0)) 0 3300))
                (proto-send b)
                (sleep 0.05))))))

(defun send-map-row (row-i)
    (progn
        (bufset-u8 row-packet 0 pkt-map-row)
        (bufset-u8 row-packet 1 row-i)
        ; Traction rows store fewer columns; the tail stays zero.
        (looprange d 0 21
            (bufset-i16 row-packet (+ 2 (* d 2))
                (if (< d (map-row-cols row-i)) (map-get-cell row-i d) 0)))
        (proto-send row-packet)))

(defun send-full-map ()
    (looprange r 0 map-thr-n
        (progn (send-map-row r) (sleep 0.002))))

(defun send-cfg-echo ()
    (let ((b cfg-packet))
        (progn (bufset-u8 b 0 pkt-cfg-echo)
        (bufset-u8 b 1 cfg-preset)
        (bufset-i16 b 2 (fx-enc cfg-torque-resp))
        (bufset-i16 b 4 (fx-enc cfg-speed-coupling))
        (bufset-i16 b 6 (fx-enc cfg-trans-width))
        (bufset-u8 b 8 cfg-trans-shape)
        (bufset-i16 b 9 (fx-enc cfg-high-hold))
        (bufset-i16 b 11 (fx-enc cfg-engine-brake))
        (bufset-i16 b 13 (fx-enc cfg-overrun-regen))
        (bufset-u8 b 15 cfg-regen-curve)
        (bufset-u8 b 16 thr-cfg-source)
        (bufset-u8 b 17 thr-cfg-invert)
        (bufset-i16 b 18 cfg-duty-filter)
        (bufset-i16 b 20 cfg-slew-ms)
        (bufset-i16 b 22 thr-cfg-deadband)
        (bufset-i16 b 24 thr-cfg-filter)
        (bufset-u8 b 26 thr-cfg-brake-mode)
        (bufset-u8 b 27 cfg-brake-map)
        (bufset-u8 b 28 cfg-brake-type)
        (bufset-i16 b 29 0)
        (bufset-i16 b 31 (fx-enc cfg-brake-str))
        (bufset-i16 b 33 (fx-enc cfg-brake-resp))
        (bufset-i16 b 35 (fx-enc cfg-brake-dep))
        (bufset-u8 b 37 cfg-brake-curve)
        (bufset-i16 b 38 (fx-enc cfg-rev-coupling))
        (bufset-i16 b 40 (fx-enc cfg-rev-width))
        (bufset-i16 b 42 (fx-enc cfg-rev-overrun))
        (bufset-i16 b 44 (fx-enc cfg-rev-str))
        (bufset-i16 b 46 (fx-enc cfg-rev-resp))
        (bufset-i16 b 48 (fx-enc cfg-rev-hold))
        (bufset-u8 b 50 cfg-rev-shape)
        (bufset-u8 b 51 cfg-rev-curve)
        (proto-send b))))

(defun packet-valid (data)
    (let ((n (buflen data)))
        (and (> n 0)
            (let ((cmd (bufget-u8 data 0)))
                (cond
                    ((= cmd pkt-set-cell)
                        (and (= n 5) (< (bufget-u8 data 1) map-thr-n)
                             (< (bufget-u8 data 2) (map-row-cols (bufget-u8 data 1)))
                             (in-range (bufget-i16 data 3) -1000 1000)))
                    ((= cmd pkt-set-map-row)
                        (and (= n 44) (< (bufget-u8 data 1) map-thr-n)
                            (let ((ok t))
                                (progn (looprange d 0 (map-row-cols (bufget-u8 data 1))
                                    (if (not (in-range (bufget-i16 data (+ 2 (* d 2))) -1000 1000))
                                        (setq ok nil)))
                                ok))))
                    ((= cmd pkt-set-config)
                        (and (= n 44) (<= (bufget-u8 data 1) 4)
                             (in-range (bufget-i16 data 2) 300 2000)
                             (in-range (bufget-i16 data 4) 0 1500)
                             (in-range (bufget-i16 data 6) 20 300)
                             (<= (bufget-u8 data 8) 3)
                             (in-range (bufget-i16 data 9) 0 1000)
                             (in-range (bufget-i16 data 11) 0 600)
                             (in-range (bufget-i16 data 13) 0 500)
                             (<= (bufget-u8 data 15) 3)
                             (<= (bufget-u8 data 16) 1)
                             (<= (bufget-u8 data 17) 1)
                             (<= (bufget-u8 data 18) 2)
                             (in-range (bufget-i16 data 19) 0 20000)
                             (in-range (bufget-i16 data 21) 0 1000)
                             (in-range (bufget-i16 data 23) 300 2000)
                             (in-range (bufget-i16 data 25) 0 1000)
                             (<= (bufget-u8 data 27) 3)
                             (<= (bufget-u8 data 28) 1)
                             (in-range (bufget-i16 data 29) 0 1500)
                             (in-range (bufget-i16 data 31) 20 300)
                             (in-range (bufget-i16 data 33) 0 500)
                             (in-range (bufget-i16 data 35) 0 1000)
                             (in-range (bufget-i16 data 37) 300 2000)
                             (in-range (bufget-i16 data 39) 0 1000)
                             (<= (bufget-u8 data 41) 3)
                             (<= (bufget-u8 data 42) 3)
                             (<= (bufget-u8 data 43) 1)))
                    ((= cmd pkt-set-thr)
                        (and (= n 12) (<= (bufget-u8 data 1) 3)
                             (<= (bufget-u8 data 2) 1)
                             (in-range (bufget-i16 data 3) 1 1000)
                             (in-range (bufget-i16 data 5) 0 2000)
                             (in-range (bufget-i16 data 7) 0 999)
                             (in-range (bufget-i16 data 9) 1 1000)
                             (<= (bufget-u8 data 11) 2)))
                    ((= cmd pkt-calibrate-bidir) (= n 1))
                    ((= cmd pkt-set-test-thr)
                        (and (= n 3) (in-range (bufget-i16 data 1) -1000 1000)))
                    (t (and (= n 1) (>= cmd pkt-cmd-save) (<= cmd pkt-req-cfg))))))))

(defun dispatch-packet (data)
    (let ((cmd (bufget-u8 data 0)))
        (cond
            ((= cmd pkt-set-cell)
                (map-set-cell (bufget-u8 data 1) (bufget-u8 data 2) (bufget-i16 data 3)))
            ((= cmd pkt-set-map-row)
                (looprange d 0 (map-row-cols (bufget-u8 data 1))
                    (map-set-cell (bufget-u8 data 1) d (bufget-i16 data (+ 2 (* d 2))))))
            ((= cmd pkt-set-config)
                (progn
                    (setq cfg-preset (bufget-u8 data 1))
                    (setq cfg-torque-resp (fx-dec (bufget-i16 data 2)))
                    (setq cfg-speed-coupling (fx-dec (bufget-i16 data 4)))
                    (setq cfg-trans-width (fx-dec (bufget-i16 data 6)))
                    (setq cfg-trans-shape (bufget-u8 data 8))
                    (setq cfg-high-hold (fx-dec (bufget-i16 data 9)))
                    (setq cfg-engine-brake (fx-dec (bufget-i16 data 11)))
                    (setq cfg-overrun-regen (fx-dec (bufget-i16 data 13)))
                    (setq cfg-regen-curve (bufget-u8 data 15))
                    (setq cfg-brake-map (bufget-u8 data 17))
                    (setq cfg-brake-type (bufget-u8 data 18))
                    (setq cfg-brake-str (fx-dec (bufget-i16 data 21)))
                    (setq cfg-brake-resp (fx-dec (bufget-i16 data 23)))
                    (setq cfg-brake-dep (fx-dec (bufget-i16 data 25)))
                    (setq cfg-brake-curve (bufget-u8 data 27))
                    (setq cfg-rev-coupling (fx-dec (bufget-i16 data 29)))
                    (setq cfg-rev-width (fx-dec (bufget-i16 data 31)))
                    (setq cfg-rev-overrun (fx-dec (bufget-i16 data 33)))
                    (setq cfg-rev-str (fx-dec (bufget-i16 data 35)))
                    (setq cfg-rev-resp (fx-dec (bufget-i16 data 37)))
                    (setq cfg-rev-hold (fx-dec (bufget-i16 data 39)))
                    (setq cfg-rev-shape (bufget-u8 data 41))
                    (setq cfg-rev-curve (bufget-u8 data 42))
                    ; Each half regenerates on its own flag: a traction
                    ; slider never rewrites brake rows, and vice versa.
                    (if (= (bufget-u8 data 16) 1)
                        (gen-thermal-map cfg-torque-resp cfg-speed-coupling cfg-trans-width
                            cfg-trans-shape cfg-high-hold cfg-engine-brake cfg-overrun-regen cfg-regen-curve))
                    (if (= (bufget-u8 data 28) 1) (gen-brake-half))
                    (if (= (bufget-u8 data 43) 1) (gen-rev-half))))
            ((= cmd pkt-set-thr)
                (progn
                    (setq thr-cfg-source (bufget-u8 data 1))
                    (setq thr-cfg-invert (bufget-u8 data 2))
                    (setq cfg-duty-filter (bufget-i16 data 3))
                    (setq cfg-slew-ms (bufget-i16 data 5))
                    (setq thr-cfg-deadband (bufget-i16 data 7))
                    (setq thr-cfg-filter (bufget-i16 data 9))
                    (setq thr-cfg-brake-mode (bufget-u8 data 11))
                    (thr-reset-state)))
            ((= cmd pkt-set-test-thr)
                (progn
                    (setq thr-test-value (bufget-i16 data 1))
                    (setq thr-test-time (systime))
                    ; STOP clears the filter immediately.
                    (if (= thr-test-value 0)
                        (progn (setq thr-filtered 0) (setq thr-filter-acc 0)))))
            ((= cmd pkt-calibrate-bidir)
                (if (and (= thr-cfg-source thr-src-adc) (= thr-cfg-brake-mode thr-brake-bidir))
                    (progn (thr-calibrate-bidir-center) (thr-reset-state))
                    (exit-error 'bidir-calibration-requires-bidir-adc)))
            ((= cmd pkt-cmd-save) (if (not (storage-save)) (exit-error 'storage-error)))
            ((= cmd pkt-cmd-load) (if (not (storage-load)) (exit-error 'storage-error)))
            ((= cmd pkt-cmd-reset) (storage-reset))
            ((= cmd pkt-req-map) (send-full-map))
            ((= cmd pkt-req-cfg) (send-cfg-echo)))))

(defun handle-packet (data)
    (if (packet-valid data)
        (let ((cmd (bufget-u8 data 0)))
            ; Pause only operations that replace configuration or a full map.
            (progn (if (and (>= cmd pkt-set-config) (<= cmd pkt-cmd-reset))
                (progn (setq storage-busy t) (sleep 0.01)))
            (let ((result (trap (dispatch-packet data))))
                (progn (setq storage-busy nil)
                (proto-send-status
                    (if (eq (car result) 'exit-ok)
                        (if (and (>= cmd pkt-cmd-save) (<= cmd pkt-cmd-reset)) (- cmd 4) 0)
                        (if (= cmd pkt-cmd-save) 4 (if (= cmd pkt-cmd-load) 5 7)))
                    cmd)))))
        (proto-send-status 6 (if (> (buflen data) 0) (bufget-u8 data 0) 0))))

(defun event-handler ()
    (loopwhile t
        (recv
            ((event-data-rx . (? data)) (handle-packet data))
            (_ nil))))
@const-end

(if (not (storage-load)) (storage-reset))
(event-register-handler (spawn "tmpro-rx" 256 event-handler))
(event-enable 'event-data-rx)
(spawn "tmpro-ctl" 150 control-loop)
(spawn "tmpro-tel" 80 telemetry-loop)
