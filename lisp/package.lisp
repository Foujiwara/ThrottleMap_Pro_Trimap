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
(define cfg-rev-erpm 500)
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
(define live-throttle 0)
(define live-duty 0)
(define live-cur-rel 0)
(define live-brake 0)
; Latched once "no reverse" braking has brought the vehicle to a stop.
(define brake-stopped nil)
; One buffer per owner: telemetry never shares its buffer with the event task.
(define row-packet (array-create 44))
(define cfg-packet (array-create 52))

@const-start
; Lever braking only. Engine braking and overrun regen always stay pure
; regen: a bidirectional brake type must never reverse a released vehicle.
(defun brake-apply (mag)
    (if (= cfg-brake-type 0)
        (set-brake-rel (fp-to-f mag))
        (let ((r (to-i (get-rpm))))
            (if (= cfg-brake-type 2)
                ; Bidirectional: negative torque below the threshold, and
                ; it keeps going once that torque has reversed the motor.
                (if (> r cfg-rev-erpm)
                    (set-brake-rel (fp-to-f mag))
                    (set-current-rel (- (fp-to-f mag))))
                (progn
                    ; No reverse: negative torque only while it is still
                    ; travelling forwards, and latched off the moment it
                    ; reaches a stop. Deciding on speed alone oscillates -
                    ; the torque reverses the motor, regen catches it, the
                    ; speed lands back in the torque band, and it shunts
                    ; backwards over and over. The latch clears when the
                    ; lever is released or it rolls forwards again.
                    (if (> r cfg-rev-erpm) (setq brake-stopped nil))
                    (if (< r 50) (setq brake-stopped t))
                    (if (or brake-stopped (> r cfg-rev-erpm))
                        (set-brake-rel (fp-to-f mag))
                        (set-current-rel (- (fp-to-f mag)))))))))

(defun control-tick ()
    (if storage-busy
        (setq live-cur-rel 0)
        (progn
            (setq live-throttle (thr-read))
            (setq live-brake (thr-brake-read))
            (setq live-duty (to-fp (get-duty)))
            (let ((lever (> live-brake 0)))
                (progn
                    (if (not lever) (setq brake-stopped nil))
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
                    ; Negative map values mean braking, not reverse propulsion.
                    (if (< live-cur-rel 0)
                        (if lever
                            (brake-apply (- live-cur-rel))
                            (set-brake-rel (fp-to-f (- live-cur-rel))))
                        (set-current-rel (fp-to-f live-cur-rel))))))))

(defun control-loop ()
    (loopwhile t (progn (control-tick) (sleep 0.005))))

(defun telemetry-loop ()
    (let ((b (array-create 15)))
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
        (bufset-i16 b 18 thr-cfg-min)
        (bufset-i16 b 20 thr-cfg-max)
        (bufset-i16 b 22 thr-cfg-deadband)
        (bufset-i16 b 24 thr-cfg-filter)
        (bufset-u8 b 26 thr-cfg-brake-mode)
        (bufset-u8 b 27 cfg-brake-map)
        (bufset-u8 b 28 cfg-brake-type)
        (bufset-i16 b 29 cfg-rev-erpm)
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
                             (in-range (bufget-i16 data 3) 0 1000)
                             (in-range (bufget-i16 data 5) 0 1000)
                             (< (bufget-i16 data 3) (bufget-i16 data 5))
                             (in-range (bufget-i16 data 7) 0 999)
                             (in-range (bufget-i16 data 9) 1 1000)
                             (<= (bufget-u8 data 11) 2)))
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
                    (setq cfg-rev-erpm (bufget-i16 data 19))
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
                    (setq thr-cfg-min (bufget-i16 data 3))
                    (setq thr-cfg-max (bufget-i16 data 5))
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
