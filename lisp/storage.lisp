; EEPROM image for the 31x11 map. The header is packed (i16 scaled by 1000
; instead of f32) so the taller map fits: 36 bytes of header + 344 bytes of
; cells = 95 of the 127 data slots, slot 127 holds CRC16 of slots 0..126.
; Images written by the 21x21 packages are migrated on read, never written.
(define storage-busy nil)

@const-start
(define eeprom-magic 20260914)
(define eeprom-legacy-magic 20260913)
(define eeprom-legacy-magic0 20260912)
(define eeprom-map-base 36)
(define eeprom-map-slots 86)

; Avoid firmware versions where bufget-i32 narrows to a 28-bit fixnum.
(defun storage-buffer-i32 (b offset) (to-i32 (bufget-u32 b offset)))

; Promote BEFORE shifting: plain LispBM integers have only 28 bits on VESC.
(defun pack4 (v0 v1 v2 v3)
    (bitwise-or
        (bitwise-or (to-u32 (+ v0 128)) (shl (to-u32 (+ v1 128)) 8))
        (bitwise-or (shl (to-u32 (+ v2 128)) 16) (shl (to-u32 (+ v3 128)) 24))))

(defun map-cell-flat-i8 (i)
    (if (< i map-cells) (bufget-i8 map-buf i) 0))

(defun storage-image ()
    (let ((b (array-create 508)))
        (progn (bufset-i32 b 0 eeprom-magic)
        (bufset-u8 b 4 thr-cfg-source)
        (bufset-u8 b 5 thr-cfg-invert)
        (bufset-u8 b 6 thr-cfg-brake-mode)
        (bufset-u8 b 7 cfg-preset)
        (bufset-i16 b 8 thr-cfg-min)
        (bufset-i16 b 10 thr-cfg-max)
        (bufset-i16 b 12 thr-cfg-deadband)
        (bufset-i16 b 14 thr-cfg-filter)
        (bufset-i16 b 16 (to-fp cfg-torque-resp))
        (bufset-i16 b 18 (to-fp cfg-speed-coupling))
        (bufset-i16 b 20 (to-fp cfg-trans-width))
        (bufset-i16 b 22 (to-fp cfg-high-hold))
        (bufset-i16 b 24 (to-fp cfg-engine-brake))
        (bufset-i16 b 26 (to-fp cfg-overrun-regen))
        (bufset-u8 b 28 cfg-trans-shape)
        (bufset-u8 b 29 cfg-regen-curve)
        (bufset-u8 b 30 cfg-brake-map)
        (bufset-u8 b 31 cfg-brake-type)
        (bufset-i16 b 32 cfg-rev-erpm)
        (looprange s 0 eeprom-map-slots
            (bufset-u32 b (+ eeprom-map-base (* s 4))
                (pack4 (map-cell-flat-i8 (* s 4))
                       (map-cell-flat-i8 (+ (* s 4) 1))
                       (map-cell-flat-i8 (+ (* s 4) 2))
                       (map-cell-flat-i8 (+ (* s 4) 3)))))
        b)))

; Readback is mandatory: some firmware paths return true without writing
; if motor release times out. Unchanged slots incur no flash writes.
(defun storage-write-word (addr value)
    (let ((old (eeprom-read-i addr)))
        (if (and (number? old) (= old value))
            t
            (and (eeprom-store-i addr value)
                (let ((actual (eeprom-read-i addr)))
                    (and (number? actual) (= actual value)))))))

(defun storage-write-image (b)
    (let ((checksum (crc16 b)) (same t))
        (progn (looprange s 0 127
            (let ((old (eeprom-read-i s)))
                (if (not (and (number? old) (= old (storage-buffer-i32 b (* s 4)))))
                    (setq same nil))))
        (if (and same (let ((old (eeprom-read-i 127)))
                          (and (number? old) (= old checksum))))
            t
            ; Invalidate FIRST, commit marker LAST. A power failure leaves
            ; an invalid image, never a mix of old and new settings.
            (and (storage-write-word 0 0)
                (let ((ok t))
                    (progn (looprange s 1 127
                        (if ok
                            (setq ok (storage-write-word s (storage-buffer-i32 b (* s 4))))))
                    (and ok (storage-write-word 127 checksum)
                         (storage-write-word 0 eeprom-magic)))))))))

(defun storage-save ()
    (progn
        (setq storage-busy t)
        (sleep 0.01)
        (let ((result (trap (storage-write-image (storage-image)))))
            (progn (setq storage-busy nil)
            (eq result '(exit-ok t))))))

; Packed slot's least significant byte is the first cell.
(defun storage-cell-at (b base i)
    (- (bufget-u8 b (+ base (* (/ i 4) 4) (- 3 (mod i 4)))) 128))

(defun storage-image-cell (b i) (storage-cell-at b eeprom-map-base i))

(defun storage-cells-valid (b base n)
    (let ((ok t))
        (progn (looprange i 0 n
            (if (not (in-range (storage-cell-at b base i) -100 100)) (setq ok nil)))
        ok)))

; Range checks reject missing fields, corrupt calibration and maps.
(defun storage-image-valid (b)
    (and
        (in-range (bufget-u8 b 4) 0 3)
        (in-range (bufget-u8 b 5) 0 1)
        (in-range (bufget-u8 b 6) 0 2)
        (in-range (bufget-u8 b 7) 0 4)
        (in-range (bufget-i16 b 8) 0 1000)
        (in-range (bufget-i16 b 10) 0 1000)
        (< (bufget-i16 b 8) (bufget-i16 b 10))
        (in-range (bufget-i16 b 12) 0 999)
        (in-range (bufget-i16 b 14) 1 1000)
        (in-range (bufget-i16 b 16) 300 2000)
        (in-range (bufget-i16 b 18) 0 1500)
        (in-range (bufget-i16 b 20) 20 300)
        (in-range (bufget-i16 b 22) 0 1000)
        (in-range (bufget-i16 b 24) 0 600)
        (in-range (bufget-i16 b 26) 0 500)
        (in-range (bufget-u8 b 28) 0 3)
        (in-range (bufget-u8 b 29) 0 3)
        (in-range (bufget-u8 b 30) 0 1)
        (in-range (bufget-u8 b 31) 0 2)
        (in-range (bufget-i16 b 32) 0 20000)
        (storage-cells-valid b eeprom-map-base map-cells)))

(defun storage-apply-image (b)
    (progn
        (setq thr-cfg-source (bufget-u8 b 4))
        (setq thr-cfg-invert (bufget-u8 b 5))
        (setq thr-cfg-brake-mode (bufget-u8 b 6))
        (setq cfg-preset (bufget-u8 b 7))
        (setq thr-cfg-min (bufget-i16 b 8))
        (setq thr-cfg-max (bufget-i16 b 10))
        (setq thr-cfg-deadband (bufget-i16 b 12))
        (setq thr-cfg-filter (bufget-i16 b 14))
        (setq cfg-torque-resp (fp-to-f (bufget-i16 b 16)))
        (setq cfg-speed-coupling (fp-to-f (bufget-i16 b 18)))
        (setq cfg-trans-width (fp-to-f (bufget-i16 b 20)))
        (setq cfg-high-hold (fp-to-f (bufget-i16 b 22)))
        (setq cfg-engine-brake (fp-to-f (bufget-i16 b 24)))
        (setq cfg-overrun-regen (fp-to-f (bufget-i16 b 26)))
        (setq cfg-trans-shape (bufget-u8 b 28))
        (setq cfg-regen-curve (bufget-u8 b 29))
        (setq cfg-brake-map (bufget-u8 b 30))
        (setq cfg-brake-type (bufget-u8 b 31))
        (setq cfg-rev-erpm (bufget-i16 b 32))
        (looprange i 0 map-cells
            (bufset-i8 map-buf i (storage-image-cell b i)))
        (thr-reset-state)
        t))

; ---- one-way migration from the 21x21 format -----------------------------

(defun storage-legacy-valid (b)
    (and
        (in-range (storage-buffer-i32 b 4) 0 3)
        (in-range (storage-buffer-i32 b 8) 0 1000)
        (in-range (storage-buffer-i32 b 12) 0 1000)
        (< (storage-buffer-i32 b 8) (storage-buffer-i32 b 12))
        (= (bitwise-and (storage-buffer-i32 b 16) 0xfe) 0)
        (in-range (shr (storage-buffer-i32 b 16) 8) 0 999)
        (in-range (storage-buffer-i32 b 20) 1 1000)
        (in-range (storage-buffer-i32 b 24) 0 4)
        (in-range (bufget-f32 b 28) 0.3 2.0)
        (in-range (bufget-f32 b 32) 0.0 1.5)
        (in-range (bufget-f32 b 36) 0.02 0.30)
        (in-range (storage-buffer-i32 b 40) 0 3)
        (in-range (bufget-f32 b 44) 0.0 1.0)
        (in-range (bufget-f32 b 48) 0.0 0.6)
        (in-range (bufget-f32 b 52) 0.0 0.5)
        (in-range (storage-buffer-i32 b 56) 0 3)
        (in-range (storage-buffer-i32 b 504) 0 2)
        (storage-cells-valid b 60 441)))

; The 21 old throttle rows become rows 10..30 unchanged; the duty axis keeps
; every other column, which lands exactly on the new 10% grid.
(defun storage-apply-legacy (b)
    (progn
        (setq thr-cfg-source (to-i (storage-buffer-i32 b 4)))
        (setq thr-cfg-min (to-i (storage-buffer-i32 b 8)))
        (setq thr-cfg-max (to-i (storage-buffer-i32 b 12)))
        (setq thr-cfg-invert (to-i (bitwise-and (storage-buffer-i32 b 16) 1)))
        (setq thr-cfg-deadband (to-i (shr (storage-buffer-i32 b 16) 8)))
        (setq thr-cfg-filter (to-i (storage-buffer-i32 b 20)))
        (setq cfg-preset (to-i (storage-buffer-i32 b 24)))
        (setq cfg-torque-resp (bufget-f32 b 28))
        (setq cfg-speed-coupling (bufget-f32 b 32))
        (setq cfg-trans-width (bufget-f32 b 36))
        (setq cfg-trans-shape (to-i (storage-buffer-i32 b 40)))
        (setq cfg-high-hold (bufget-f32 b 44))
        (setq cfg-engine-brake (bufget-f32 b 48))
        (setq cfg-overrun-regen (bufget-f32 b 52))
        (setq cfg-regen-curve (to-i (storage-buffer-i32 b 56)))
        (setq thr-cfg-brake-mode (to-i (storage-buffer-i32 b 504)))
        (setq cfg-brake-map 0)
        (setq cfg-brake-type 0)
        (setq cfg-rev-erpm 500)
        (looprange ti 0 21
            (looprange di 0 11
                (bufset-i8 map-buf (map-idx (+ ti map-thr-zero) di)
                    (storage-cell-at b 60 (+ (* ti 21) (* di 2))))))
        (gen-brake-map)
        (thr-reset-state)
        t))

(defun storage-read-image ()
    (let ((magic (eeprom-read-i 0)))
        ; eq would reject the i32 returned by EEPROM against a plain integer.
        (if (not (number? magic))
            nil
            (let ((legacy (or (= magic eeprom-legacy-magic)
                              (= magic eeprom-legacy-magic0))))
                (if (not (or legacy (= magic eeprom-magic)))
                    nil
                    (let ((b (array-create 508)) (ok t))
                        (progn (looprange s 0 127
                            (let ((v (eeprom-read-i s)))
                                (if (number? v)
                                    (bufset-i32 b (* s 4) v)
                                    (setq ok nil))))
                        (cond
                            ((not ok) nil)
                            (legacy (if (storage-legacy-valid b)
                                        (storage-apply-legacy b) nil))
                            ((and (storage-image-valid b)
                                  (let ((sum (eeprom-read-i 127)))
                                      (and (number? sum) (= sum (crc16 b)))))
                                (storage-apply-image b))
                            (t nil)))))))))

(defun storage-load ()
    (progn
        (setq storage-busy t)
        (sleep 0.01)
        (let ((result (trap (storage-read-image))))
            (progn (setq storage-busy nil)
            (eq result '(exit-ok t))))))

(defun storage-reset ()
    (progn
        (setq cfg-preset 1)
        (setq cfg-torque-resp 0.85)
        (setq cfg-speed-coupling 1.0)
        (setq cfg-trans-width 0.10)
        (setq cfg-trans-shape 1)
        (setq cfg-high-hold 0.55)
        (setq cfg-engine-brake 0.15)
        (setq cfg-overrun-regen 0.12)
        (setq cfg-regen-curve 1)
        (setq cfg-brake-map 0)
        (setq cfg-brake-type 0)
        (setq cfg-rev-erpm 500)
        (setq thr-cfg-source thr-src-adc)
        (setq thr-cfg-invert 0)
        (setq thr-cfg-min 20)
        (setq thr-cfg-max 980)
        (setq thr-cfg-deadband 20)
        (setq thr-cfg-filter 1000)
        (setq thr-cfg-brake-mode 0)
        (thr-reset-state)
        (gen-thermal-map cfg-torque-resp cfg-speed-coupling cfg-trans-width
                         cfg-trans-shape cfg-high-hold cfg-engine-brake
                         cfg-overrun-regen cfg-regen-curve)
        (gen-brake-map)
        t))
@const-end
