"""Concatenate the mocks and every lisp module into one test program.

The package ships imported modules; the upstream LispBM test runner takes a
single file, so the bodies are concatenated here. Flash directives are kept
intact so the const regions are exercised the way they are on hardware.

package.lisp is cut at its "; ---- boot ----" marker: everything below it
spawns threads or touches hardware, and the suite drives control-tick on its
own. Comments are stripped so a ';' inside them can never be read as code.

Was prepare-lisp.mjs; Python because the lisp half of the suite should not
need a Node install to run.
"""
import io
import os
import re
import sys

os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
os.makedirs(".test-runtime", exist_ok=True)

BOOT_MARKER = "; ---- boot ----"

parts = [io.open("tests/vesc-mocks.lisp", encoding="utf-8").read()]

for module in ["util", "map", "throttle", "storage", "protocol", "package"]:
    code = io.open("lisp/%s.lisp" % module, encoding="utf-8").read()
    if module == "package":
        start = code.index("(define cfg-preset")
        end = code.find(BOOT_MARKER)
        if end < 0:
            sys.exit("package.lisp has no '%s' marker" % BOOT_MARKER)
        code = code[start:end]
    code = re.sub(r";[^\n]*", "", code).replace("\r", "")
    parts.append(code)

parts.append(io.open("tests/regression.lisp", encoding="utf-8").read())

out = ".test-runtime/regression.lisp"
io.open(out, "w", encoding="utf-8", newline="\n").write("\n".join(parts) + "\n")
print("wrote %s (%d bytes)" % (out, os.path.getsize(out)))
