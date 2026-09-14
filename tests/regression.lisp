; Regression suite. Runs the real application code in the real LispBM
; interpreter; only the hardware boundary is mocked (see vesc-mocks.lisp).
;
; Written for the ragged 451-cell grid and the current protocol. None of the
; lock assertions below existed when the position lock shipped a current
; request it never sent to the motor, so that case is covered first.

(expect (not (eq (to-i32 20260912) 20260912)) 'typed-equality-regression)
(expect (= (pack4 100 0 -100 99) 0xe31c80e4u32) 'pack-all-32-bits)
(setq boot-busy nil)

; ---- geometry ------------------------------------------------------------
(expect (= (buflen map-buf) 451) 'ragged-map-size)
(expect (= map-cells 451) 'cell-count)
; Every (row, column) that exists must map to a distinct slot, and no row may
; address a slot outside the buffer. A ragged index function is exactly the
; kind that silently overlaps two rows.
(define seen (array-create 451))
(looprange r 0 map-thr-n
    (looprange d 0 (map-row-cols r)
        (let ((i (map-idx r d)))
            (progn
                (expect (in-range i 0 450) (list 'index-out-of-range r d i))
                (expect (= (bufget-u8 seen i) 0) (list 'index-collision r d i))
                (bufset-u8 seen i 1)))))
(looprange i 0 451 (expect (= (bufget-u8 seen i) 1) (list 'unreachable-cell i)))
(expect (= (map-row-cols 10) 21) 'released-row-is-full-width)
(expect (= (map-row-cols 11) 11) 'throttle-rows-are-half-width)

; ---- storage -------------------------------------------------------------
(expect (not (storage-load)) 'empty-storage)
(storage-reset)
(expect (eq (car (trap (map-set-cell 30 10 1000))) 'exit-ok) 'map-is-writable)
(looprange i 0 map-cells (bufset-i8 map-buf i (- (mod i 201) 100)))
(setq thr-cfg-source 2)
(setq thr-cfg-invert 1)
(setq thr-cfg-deadband 50)
(setq thr-cfg-filter 125)
(setq thr-cfg-brake-mode 2)
(setq thr-bidir-center 1700)
(setq cfg-duty-filter 250)
(setq cfg-brake-map 1)
(setq cfg-brake-type 2)
(setq cfg-preset 0)
(expect (storage-save) 'first-save)
(define first-writes writes)
(expect (storage-save) 'unchanged-save)
(expect (= writes first-writes) 'unchanged-save-no-flash-writes)
(storage-reset)
(looprange i 0 map-cells
    (expect (in-range (storage-image-cell ee i) -100 100)
            (list 'bad-map-cell i (storage-image-cell ee i))))
(expect (storage-image-valid ee) 'stored-image-ranges)
(expect (storage-load) 'cold-reload)
(expect (and (= thr-cfg-source 2) (= thr-cfg-invert 1) (= thr-cfg-deadband 50)
             (= thr-cfg-filter 125) (= thr-cfg-brake-mode 2)
             (= thr-bidir-center 1700) (= cfg-duty-filter 250)
             (= cfg-brake-map 1) (= cfg-brake-type 2) (= cfg-preset 0))
        'config-roundtrip)
(expect (eq (type-of thr-cfg-filter) 'type-i) 'loaded-values-remain-fixnums)
(looprange i 0 map-cells
    (expect (= (bufget-i8 map-buf i) (- (mod i 201) 100)) 'map-roundtrip))

; The master switch is a runtime override and must never come back from the
; image - an image that can store it disabled boots the package silent, with
; no motor command at all and nothing to say why.
(setq pkg-enabled 0)
(expect (storage-save) 'save-while-disabled)
(setq pkg-enabled 1)
(expect (storage-load) 'reload-after-disabled-save)
(expect (= pkg-enabled 1) 'master-switch-not-persisted)

(define old-cell (map-get-cell 0 0))
(bufset-i32 ee 60 0)
(expect (not (storage-load)) 'crc-rejects-corruption)
(expect (= (map-get-cell 0 0) old-cell) 'failed-load-keeps-live-map)
(expect (storage-save) 'repair-corrupt-save)

; An old magic over a body that is not the old format must be rejected, not
; reinterpreted. The 21x21 header was f32 where this one is packed i16, so a
; misread would silently install nonsense settings and a garbage map.
(define live-before (map-get-cell 5 5))
(bufset-i32 ee 0 eeprom-legacy-magic0)
(expect (not (storage-load)) 'legacy-magic-wrong-body-rejected)
(expect (= (map-get-cell 5 5) live-before) 'rejected-load-keeps-live-map)
(bufset-i32 ee 0 eeprom-prev-magic)
(expect (not (storage-load)) 'prev-magic-wrong-body-rejected)
(bufset-i32 ee 0 12345678)
(expect (not (storage-load)) 'unknown-magic-rejected)
(expect (storage-save) 'save-restores-the-marker)
(expect (= (eeprom-read-i 0) eeprom-magic) 'new-format-marker)

; Interruptions and false-success firmware returns never acknowledge success.
(looprange mode 0 3
    (progn
        ; Must be a field that lands in the slot the failure is injected
        ; into, or the write is skipped as unchanged and nothing fails.
        ; cfg-duty-filter is image byte 8, which is slot 2.
        (setq cfg-duty-filter (+ 100 mode))
        (setq fail-addr 2)
        (setq fail-mode mode)
        (expect (not (storage-save)) 'write-failure-detected)
        (expect (not storage-busy) 'pause-cleared-after-error)
        (expect (not (storage-load)) 'partial-image-rejected)
        (setq fail-addr -1)
        (expect (storage-save) 'save-recovery)))
(bufset-u8 ee-valid 3 0)
(expect (not (storage-load)) 'missing-slot)
(expect (storage-save) 'restore-slot)

; ---- map lookup ----------------------------------------------------------
(storage-reset)
; Traction is read mirrored at negative duty: for the throttle, the sign of
; duty is speed, not direction.
(expect (= (map-lookup 600 400) (map-lookup 600 -400)) 'traction-mirrors-duty)
(expect (= (map-lookup 1000 1000) (map-lookup 1000 -1000)) 'mirror-at-full-duty)
; Inputs outside the grid clamp instead of running off the end of a row.
(expect (= (map-lookup 2000 3000) (map-lookup 1000 1000)) 'lookup-upper-bound)
(expect (= (map-lookup -2000 -3000) (map-lookup -1000 -1000)) 'lookup-lower-bound)
; A cell written by hand must be what the lookup returns on that exact node.
(map-set-cell 30 10 770)
(expect (= (map-lookup 1000 1000) 770) 'lookup-reads-the-cell)
(map-set-cell 30 0 250)
(expect (= (map-lookup 1000 0) 250) 'lookup-reads-zero-duty-cell)
; Halfway between two nodes must land halfway between their values.
(map-set-cell 30 1 350)
(expect (= (map-lookup 1000 50) 300) 'lookup-interpolates)
; The brake half is a separate input and never blends into traction.
(map-set-cell 0 0 -900)
(expect (= (map-lookup -1000 -1000) -900) 'brake-half-reads-its-own-cell)

; ---- position lock -------------------------------------------------------
; The lock must actually command the motor. It once computed a request,
; published it in telemetry and never called set-current-rel at all: let
; takes one body form, and the call sat in a second one that never ran.
(storage-reset)
(setq rpm-value 0.0)
(setq dist-value 0.0)
(setq adc-value 0.0)
(setq thr-cfg-source 0)
(setq thr-cfg-invert 0)
(setq thr-cfg-brake-mode 0)
(setq thr-cfg-filter 1000)
(setq cfg-lock-free 0)
(setq cfg-lock-travel 250)
(setq cfg-lock-max 150)
(setq cfg-lock-damp 0)
(setq cfg-lock-timeout 60)
(expect (lock-engage) 'engage-at-standstill)
(expect lock-on 'lock-is-on)
(expect (= lock-mmrev 797) 'scale-from-wheel-config)
(expect (= lock-span 199) 'span-from-travel)
; Pushed a quarter turn forward: the ceiling, negative, and actually sent.
(setq dist-value 0.2)
(setq motor-calls 0)
(control-tick)
(expect (> motor-calls 0) 'lock-commands-the-motor)
(expect (not brake-command) 'lock-uses-current-not-brake)
(expect (= live-cur-rel -150) 'lock-saturates-at-ceiling)
(expect (< motor-value 0.0) 'lock-pushes-back)
; Pushed the other way it must push the other way too. A brake command
; cannot do that, which is why the lock never uses one.
(setq dist-value -0.2)
(control-tick)
(expect (= live-cur-rel 150) 'lock-is-symmetric)
; Half the spring travel is half the effort.
(setq dist-value 0.1)
(control-tick)
(expect (= live-cur-rel -75) 'lock-is-proportional)
; Dead travel is free: no spring, no damping, nothing commanded.
(setq cfg-lock-free 500)
(lock-set-span)
(expect (= lock-dead 398) 'dead-travel-in-mm)
(setq dist-value 0.3)
(control-tick)
(expect (= live-cur-rel 0) 'inside-dead-travel-is-free)
(setq dist-value 0.5)
(control-tick)
(expect (< live-cur-rel 0) 'past-dead-travel-the-spring-acts)
; The spring starts from zero at the edge of the dead travel, not from the
; value it would have had without one.
(setq dist-value 0.398)
(control-tick)
(expect (= live-cur-rel 0) 'spring-starts-at-zero)
(setq cfg-lock-free 0)
(lock-set-span)
; Damping opposes speed on its own, with no deflection at all.
(setq dist-value 0.0)
(setq cfg-lock-damp 300)
(setq rpm-value 1000.0)
(control-tick)
(expect (= live-cur-rel -150) 'damping-opposes-motion)
(setq rpm-value -1000.0)
(control-tick)
(expect (= live-cur-rel 150) 'damping-is-symmetric)
(setq cfg-lock-damp 0)
(setq rpm-value 0.0)
; Throttle releases it, and records why.
(setq thr-cfg-source 3)
(setq thr-test-value 500)
(setq thr-test-time clock-ms)
(control-tick)
(expect (not lock-on) 'throttle-releases-the-lock)
(expect (= lock-reason 2) 'release-reason-throttle)
(setq thr-test-value 0)
(setq thr-cfg-source 0)
(setq adc-value 0.0)
(control-tick)
; It can never be armed while moving.
(setq rpm-value 500.0)
(expect (not (lock-engage)) 'engage-refused-while-moving)
(setq rpm-value 0.0)
(expect (lock-engage) 'engage-allowed-again)
; The hold time is a limit, not a fixture, and 0 removes it.
(setq cfg-lock-timeout 1)
(setq clock-ms (+ clock-ms 2000))
(control-tick)
(expect (not lock-on) 'hold-time-releases)
(expect (= lock-reason 1) 'release-reason-timeout)
(setq cfg-lock-timeout 0)
(expect (lock-engage) 're-engage-without-limit)
(setq clock-ms (+ clock-ms 600000))
(control-tick)
(expect lock-on 'zero-hold-time-never-releases)
; Disabling the package drops the lock and stops commanding entirely.
(setq motor-calls 0)
(setq pkg-enabled 0)
(control-tick)
(expect (= motor-calls 0) 'disabled-commands-nothing)
(setq pkg-enabled 1)
(lock-release)
(setq cfg-lock-timeout 60)

; ---- throttle ------------------------------------------------------------
(storage-reset)
(setq thr-cfg-deadband 20)
(setq thr-cfg-filter 100)
(setq adc-value 1.0)
(looprange i 0 100 (thr-read))
(setq adc-value 0.0)
(looprange i 0 200 (thr-read))
(expect (= thr-filtered 0) 'ema-settles-to-zero)
(setq thr-cfg-invert 1)
(expect (= (thr-normalize 990) 0) 'inverted-deadband)
(setq thr-cfg-invert 0)
(setq thr-cfg-source 3)
(setq thr-test-value 800)
(setq thr-test-time clock-ms)
(setq thr-cfg-filter 1000)
(expect (= (thr-read) 800) 'bench-throttle)
(setq clock-ms (+ clock-ms 501))
(control-tick)
(expect (= motor-value 0.0) 'bench-watchdog)
(setq thr-cfg-source 1)
(setq ppm-age 0.6)
(setq adc-value 1.0)
(control-tick)
(expect (= motor-value 0.0) 'ppm-watchdog)
(setq ppm-age 0.0)
(uart-throttle-byte 0x33)
(uart-throttle-byte 0xA5)
(uart-throttle-byte 100)
(uart-throttle-byte (bitwise-xor 0xA5 100))
(expect (= uart-last-raw 500) 'uart-fragmented-frame)
(setq thr-cfg-source 0)
(setq thr-cfg-brake-mode 2)
(setq adc-value 0.5)
(setq adc-reads 0)
(control-tick)
(expect (= adc-reads 1) 'single-bidir-sample)

; ---- brake routing -------------------------------------------------------
; Braking is torque against the way the machine is actually moving. A reverse
; request from a standstill is propulsion and must leave as current even in
; regen-only, or the vehicle simply cannot back up.
(setq cfg-brake-type 0)
(setq rpm-value 1000.0)
(apply-output -500)
(expect brake-command 'braking-while-rolling-uses-brake-api)
(setq rpm-value 0.0)
(apply-output -500)
(expect (not brake-command) 'reverse-from-standstill-is-current)
(setq cfg-brake-type 1)
(apply-output -500)
(expect (= motor-value 0.0) 'no-reverse-holds-zero)
(setq cfg-brake-type 2)
(apply-output -500)
(expect (< motor-value 0.0) 'bidirectional-reverses)
(setq cfg-brake-type 0)
(setq rpm-value 0.0)

; ---- protocol ------------------------------------------------------------
(define packet (array-create 3))
(bufset-u8 packet 0 0x03)
(handle-packet packet)
(expect (= last-status 6) 'short-packet-rejected)
(bufset-u8 packet 0 0x0A)
(bufset-i16 packet 1 0)
(handle-packet packet)
(expect (= last-status 0) 'receiver-survives-malformed-packet)

; The lock settings packet is 10 bytes and its ranges are enforced.
(define lockpkt (array-create 10))
(bufset-u8 lockpkt 0 0x0D)
(bufset-u8 lockpkt 1 45)
(bufset-i16 lockpkt 2 500)
(bufset-i16 lockpkt 4 200)
(bufset-i16 lockpkt 6 400)
(bufset-i16 lockpkt 8 250)
(handle-packet lockpkt)
(expect (= last-status 0) 'lock-config-accepted)
(expect (and (= cfg-lock-timeout 45) (= cfg-lock-travel 500)
             (= cfg-lock-max 200) (= cfg-lock-damp 400) (= cfg-lock-free 250))
        'lock-config-applied)
(bufset-i16 lockpkt 4 900)
(handle-packet lockpkt)
(expect (= last-status 6) 'lock-config-range-rejected)
(expect (= cfg-lock-max 200) 'lock-config-no-mutation-on-reject)
; Engaging over the wire is refused while moving, and reported as a failure.
(define lockcmd (array-create 2))
(bufset-u8 lockcmd 0 0x0E)
(bufset-u8 lockcmd 1 1)
; The brake-routing block above drove apply-output directly, so the live
; inputs still hold whatever the throttle block left there. Clear them, or
; the refusal below would pass for the wrong reason.
(setq live-throttle 0)
(setq live-brake 0)
(setq rpm-value 800.0)
(handle-packet lockcmd)
(expect (= last-status 7) 'engage-over-wire-refused-while-moving)
(setq rpm-value 0.0)
(handle-packet lockcmd)
(expect (= last-status 0) 'engage-over-wire-accepted)
(expect lock-on 'engaged-over-wire)
(bufset-u8 lockcmd 1 0)
(handle-packet lockcmd)
(expect (not lock-on) 'released-over-wire)
(expect (= lock-reason 5) 'release-reason-asked)

; Nothing is answered, and nothing is applied, while the script is booting.
(setq boot-busy t)
(setq cfg-lock-max 200)
(bufset-i16 lockpkt 4 250)
(handle-packet lockpkt)
(expect (= last-status 7) 'booting-rejects-packets)
(expect (= cfg-lock-max 200) 'booting-applies-nothing)
(setq boot-busy nil)

; A full configuration packet regenerates the map; an import preserves it.
(define cfg (array-create 44))
(bufset-u8 cfg 0 3)
(bufset-u8 cfg 1 0)
(bufset-i16 cfg 2 1000)
(bufset-i16 cfg 4 0)
(bufset-i16 cfg 6 20)
(bufset-u8 cfg 8 0)
(bufset-i16 cfg 9 1000)
(bufset-i16 cfg 11 20)
(bufset-i16 cfg 13 0)
(bufset-u8 cfg 15 0)
(bufset-u8 cfg 16 0)
(bufset-u8 cfg 17 0)
(bufset-u8 cfg 18 0)
(bufset-i16 cfg 19 1650)
(bufset-i16 cfg 21 300)
(bufset-i16 cfg 23 1000)
(bufset-i16 cfg 25 1000)
(bufset-u8 cfg 27 0)
(bufset-u8 cfg 28 0)
(bufset-i16 cfg 29 1000)
(bufset-i16 cfg 31 100)
(bufset-i16 cfg 33 120)
(bufset-i16 cfg 35 0)
(bufset-i16 cfg 37 1000)
(bufset-i16 cfg 39 0)
(bufset-u8 cfg 41 1)
(bufset-u8 cfg 42 1)
(bufset-u8 cfg 43 0)
; Byte 16 is the traction regenerate flag; bytes 28 and 43 are the brake and
; reverse ones. Each half rebuilds on its own, so a traction slider never
; rewrites brake rows.
(bufset-u8 cfg 16 1)
(handle-packet cfg)
(expect (= last-status 0) 'custom-config-accepted)
(define regenerated (map-get-cell 30 10))
(map-set-cell 30 10 -1000)
(bufset-u8 cfg 16 0)
(handle-packet cfg)
(expect (= (map-get-cell 30 10) -1000) 'import-preserves-map)
(bufset-u8 cfg 16 1)
(handle-packet cfg)
(expect (= (map-get-cell 30 10) regenerated) 'regenerate-rebuilds-map)
(bufset-i16 cfg 2 -1)
(handle-packet cfg)
(expect (= last-status 6) 'bad-config-rejected)
(expect (= cfg-torque-resp 1.0) 'bad-config-no-mutation)
(bufset-i16 cfg 2 1000)

; ---- stacks --------------------------------------------------------------
; The production stack sizes, doing the production work.
(spawn-trap "event-stack-test" 256
    (lambda () (progn (handle-packet cfg) (send-cfg-echo) (send-full-map)
                      (expect (storage-save) 'event-stack-save))))
(recv ((exit-ok (? pid) (? value)) t) ((exit-error (? pid) (? value)) (exit-error value)))
(define telemetry-test (spawn-trap "telemetry-stack-test" 80 telemetry-loop))
(sleep 0.12)
(kill telemetry-test t)
(recv ((exit-ok (? pid) (? value)) t) ((exit-error (? pid) (? value)) (exit-error value)))
; 10,000 ticks with the lock engaged. The lock path allocates on every tick
; and had never run under test at all.
(setq rpm-value 0.0)
(setq dist-value 0.0)
(setq cfg-lock-timeout 0)
(lock-engage)
(spawn-trap "control-stack-lock" 150 (lambda () (looprange n 0 10000 (control-tick))))
(recv ((exit-ok (? pid) (? value)) t) ((exit-error (? pid) (? value)) (exit-error value)))
(lock-release)
(spawn-trap "control-stack-test" 150 (lambda () (looprange n 0 10000 (control-tick))))
(recv ((exit-ok (? pid) (? value)) t) ((exit-error (? pid) (? value)) (exit-error value)))
(gc)
(check (list 'heap-used (lbm-heap-state 'get-num-alloc-cells) 'memory-free-words (mem-num-free)) t)
(check t)
