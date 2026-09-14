"""Structural check for the lisp sources.

Balanced parentheses prove nothing here. LispBM's let, lambda, defun and
the loop forms each take exactly ONE body form: a second one parses fine,
balances fine, and is silently never evaluated. That is how the position
lock came to publish a current request it never sent to the motor.

Exits non-zero if any form carries an extra body.
"""
import io, os, sys
os.chdir(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
BS = chr(92)

def tokenize(s):
    toks = []; i = 0; line = 1
    while i < len(s):
        c = s[i]
        if c == '\n': line += 1; i += 1; continue
        if c in ' \t\r': i += 1; continue
        if c == ';':
            while i < len(s) and s[i] != '\n': i += 1
            continue
        if c == '"':
            j = i+1
            while j < len(s):
                if s[j] == BS: j += 2; continue
                if s[j] == '"': break
                j += 1
            toks.append(('atom', s[i:j+1], line)); i = j+1; continue
        if c in '()':
            toks.append((c, c, line)); i += 1; continue
        j = i
        while j < len(s) and s[j] not in ' \t\r\n();': j += 1
        toks.append(('atom', s[i:j], line)); i = j
    return toks

def parse(toks):
    pos = [0]
    def rd():
        t = toks[pos[0]]; pos[0] += 1
        if t[0] == '(':
            lst = []
            while toks[pos[0]][0] != ')':
                lst.append(rd())
            pos[0] += 1
            return ('list', lst, t[2])
        return ('atom', t[1], t[2])
    out = []
    while pos[0] < len(toks):
        out.append(rd())
    return out

# forms whose body is a SINGLE expression in LispBM
SINGLE = {'let': 2, 'lambda': 2, 'defun': 3, 'loopwhile': 2, 'looprange': 4,
          'loopfor': 6, 'if': None}
failed = False
bad = []
def walk(n, path):
    if n[0] != 'list': return
    items = n[1]
    if items and items[0][0] == 'atom':
        head = items[0][1]
        if head in ('let', 'lambda'):
            if len(items) > 3:
                bad.append((n[2], head, len(items)-2))
        elif head == 'defun':
            if len(items) > 4:
                bad.append((n[2], head, len(items)-3))
        elif head == 'loopwhile':
            if len(items) > 3:
                bad.append((n[2], head, len(items)-2))
        elif head == 'looprange':
            if len(items) > 5:
                bad.append((n[2], head, len(items)-4))
        elif head == 'if':
            if len(items) != 4 and len(items) != 3:
                bad.append((n[2], 'if', len(items)-1))
    for c in items:
        walk(c, path)

for p in ['lisp/package.lisp','lisp/map.lisp','lisp/storage.lisp',
          'lisp/protocol.lisp','lisp/throttle.lisp','lisp/util.lisp']:
    s = io.open(p, encoding='utf-8').read()
    s = s.replace('@const-start','').replace('@const-end','')
    bad = []
    for form in parse(tokenize(s)):
        walk(form, p)
    if bad:
        failed = True
        for ln, h, n in bad:
            print("%s:%d  %s has %d body forms (only the first runs)" % (p, ln, h, n))
    else:
        print(p, 'clean')

sys.exit(1 if failed else 0)
