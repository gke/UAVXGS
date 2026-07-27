#!/usr/bin/env python3
"""
Convert pipe tables in markdown files to LaTeX longtable environments
with explicit column widths. Also auto-detects orientation:

  - If any table exceeds PORTRAIT_THRESHOLD_CM, the YAML front
    matter is set to landscape (8pt font, 0.5in margins).
  - Otherwise portrait (10pt font, 1in margins).

Usage:
  pipe2latex.py file.md [file.md ...]
"""

import re, sys

FIRST_W = '3.0cm'

PORTRAIT_THRESHOLD_CM = 16.0   # leave 0.5cm breathing room from 16.5cm textwidth
PORTRAIT_TW = 16.5             # portrait textwidth (1in margins, letter)
LANDSCAPE_TW = 24.1            # landscape textwidth (0.5in margins, letter)

def _cm(s):
    """Strip 'cm' suffix from a string like '1.4cm' and return float."""
    return float(s.rstrip('cm'))

def _col_widths(n_cols):
    """Return (first_w_str, data_w_str) for n_cols columns.

    All non-first columns get equal width distributed across available
    text width. Small tables (fits in portrait) get wider columns.
    """
    fw = _cm(FIRST_W)
    # Estimate orientation: minimum viable data width is 1.4cm
    if n_cols <= 1:
        return FIRST_W, FIRST_W
    if fw + (n_cols - 1) * 1.4 > PORTRAIT_THRESHOLD_CM:
        avail = LANDSCAPE_TW
    else:
        avail = PORTRAIT_TW
    dw = max(1.4, (avail - fw) / (n_cols - 1))
    return FIRST_W, f'{dw:.4g}cm'

def _data_width_cm(n_cols):
    """Data column width in cm for n_cols (used for landscape detection)."""
    _, dw_str = _col_widths(n_cols)
    return _cm(dw_str)

def table_width_cm(n_cols):
    """Total table width in cm for a table with n_cols columns."""
    if n_cols <= 1:
        return _cm(FIRST_W)
    fw = _cm(FIRST_W)
    dw = _data_width_cm(n_cols)
    return fw + (n_cols - 1) * dw

def count_longtable_cols(text):
    """Count columns in each \\begin{longtable}{...} environment."""
    counts = []
    pos = 0
    while True:
        m = re.search(r'\\begin\{longtable\}\{', text[pos:])
        if not m:
            break
        start = pos + m.end()  # position after the opening {
        # find matching closing brace (handle nested braces)
        depth = 1
        i = start
        while i < len(text) and depth > 0:
            if text[i] == '{':
                depth += 1
            elif text[i] == '}':
                depth -= 1
            i += 1
        spec = text[start:i-1]
        cols = len(re.findall(r'p\{', spec))
        counts.append(cols)
        pos += i
    return counts

def needs_landscape(text):
    """Return True if any longtable in text exceeds portrait width."""
    counts = count_longtable_cols(text)
    for n in counts:
        w = table_width_cm(n)
        if w > PORTRAIT_THRESHOLD_CM:
            return True
    return False

def update_orientation_yaml(text, landscape):
    """Add or update YAML front matter for portrait / landscape.

    Returns modified text. If landscape is True, geometry is set to
    ``margin=0.5in, landscape=true`` with 8pt font; otherwise
    ``margin=1in`` with 10pt font.
    """
    geom = 'geometry: margin=0.5in, landscape=true' if landscape else 'geometry: margin=1in'
    size = 'fontsize: 8pt' if landscape else 'fontsize: 10pt'

    has_front = text.startswith('---\n')
    if not has_front:
        # Insert new front matter at top
        return f'---\n{geom}\n{size}\n---\n{text}'

    # Replace existing front matter
    m = re.match(r'^---\n(.*?)\n---', text, re.DOTALL)
    if not m:
        # Malformed YAML delimiter — just prepend orientation before first ---
        return f'{geom}\n{size}\n{text}'
    yaml_block = m.group(1)
    lines = yaml_block.split('\n')
    new_lines = []
    geom_done = False
    size_done = False
    for line in lines:
        if line.startswith('geometry:'):
            new_lines.append(geom)
            geom_done = True
        elif line.startswith('fontsize:'):
            new_lines.append(size)
            size_done = True
        else:
            new_lines.append(line)
    if not geom_done:
        # Insert geometry before fontsize if fontsize exists, else append
        if not size_done:
            new_lines.append(geom)
        else:
            # Insert just before fontsize
            idx = next(i for i, l in enumerate(new_lines) if l.startswith('fontsize:'))
            new_lines.insert(idx, geom)
    if not size_done:
        new_lines.append(size)
    new_yaml = '\n'.join(new_lines)
    return f'---\n{new_yaml}\n---{text[m.end():]}'

def md2latex(text):
    """Convert markdown text to LaTeX-safe text."""
    # Escape underscores (param names like ROLL_RATE_KP), but not already-escaped ones
    text = re.sub(r'(?<!\\)_', r'\_', text)
    # Bold: **text** → \textbf{text}
    text = re.sub(r'\*\*(.+?)\*\*', r'\\textbf{\1}', text)
    # Degree symbol: ° → \textdegree
    text = text.replace('°', r'\textdegree')
    # Italic: *text* → \textit{text} (but only if not already processed)
    text = re.sub(r'(?<!\*)\*(?!\*)(.+?)(?<!\*)\*(?!\*)', r'\\textit{\1}', text)
    # Caret (for formula): P × → P $\\times$
    text = text.replace('×', r'$\times$')
    return text

def split_row(line):
    """Split a pipe table row into cells, handling optional trailing pipe."""
    parts = line.split('|')
    # If line ends with '|', the split produces a trailing empty string
    if line.strip().endswith('|'):
        return parts[1:-1]
    else:
        return parts[1:]

def pipe2latex(pipe_text):
    lines = [l for l in pipe_text.split('\n') if l.strip()]
    if len(lines) < 2:
        return pipe_text

    header = [md2latex(c.strip()) for c in split_row(lines[0])]
    sep = lines[1]

    # Determine alignment from separator
    alignments = []
    parts = split_row(sep)
    for p in parts:
        s = p.strip()
        if s.startswith(':') and s.endswith(':'):
            alignments.append('c')
        elif s.endswith(':'):
            alignments.append('r')
        else:
            alignments.append('l')

    data = []
    for line in lines[2:]:
        cells = [c.strip() for c in split_row(line)]
        if len(cells) >= len(header):
            data.append([md2latex(c) for c in cells])

    n = len(header)
    fw_str, dw_str = _col_widths(n)
    spec_parts = []
    for i in range(n):
        if i == 0:
            spec_parts.append(f'>{{\\raggedright\\arraybackslash}}p{{{fw_str}}}')
        else:
            spec_parts.append(f'>{{\\raggedleft\\arraybackslash}}p{{{dw_str}}}')
    colspec = ' '.join(spec_parts)

    out = []
    out.append(r'\begin{longtable}{' + colspec + '}')
    out.append(r'\toprule')
    out.append(' & '.join(header) + r' \\')
    out.append(r'\midrule')
    out.append(r'\endhead')

    for row in data:
        out.append(' & '.join(row) + r' \\')

    out.append(r'\bottomrule')
    out.append(r'\end{longtable}')

    return '\n'.join(out)

def process_file(path):
    with open(path) as f:
        text = f.read()

    # First pass: fix column specs in existing \begin{longtable}{...}
    def fix_colspec(m):
        spec = m.group(1)
        parts = spec.split()
        n = len(parts)
        fw_str, dw_str = _col_widths(n)
        new_parts = []
        for i in range(n):
            if i == 0:
                new_parts.append(f'>{{\\raggedright\\arraybackslash}}p{{{fw_str}}}')
            else:
                new_parts.append(f'>{{\\raggedleft\\arraybackslash}}p{{{dw_str}}}')
        return r'\begin{longtable}{' + ' '.join(new_parts) + '}'
    text = re.sub(r'\\begin\{longtable\}\{((?:[^{}]|\{[^{}]*\})*)\}', fix_colspec, text)

    # Second pass: escape special chars inside LaTeX longtable environments
    def esc_lt_chars(m):
        content = m.group(0)
        # Escape underscores (e.g. Ecks_800g → Ecks\_800g)
        content = re.sub(r'(?<!\\)_', r'\_', content)
        # Escape percent signs (e.g. ~55% → ~55\%) to prevent LaTeX comments
        content = re.sub(r'(?<!\\)%', r'\%', content)
        return content
    text = re.sub(r'(?s)\\begin\{longtable\}.*?\\end\{longtable\}', esc_lt_chars, text)

    # Third pass: convert any remaining pipe tables to LaTeX
    lines = text.split('\n')
    result = []
    i = 0
    while i < len(lines):
        stripped = lines[i].strip()
        if stripped.startswith('|') and '---' not in stripped:
            table_start = i
            while i < len(lines):
                s = lines[i].strip()
                if not s.startswith('|') and s != '':
                    break
                i += 1
            table_block = '\n'.join(lines[table_start:i])
            if table_block.count('|') >= 3:
                latex = pipe2latex(table_block)
                result.append(latex)
            else:
                result.append(table_block)
            continue
        else:
            result.append(lines[i])
        i += 1

    text = '\n'.join(result) + '\n'

    # Auto-detect orientation and update YAML front matter
    landscape = needs_landscape(text)
    text = update_orientation_yaml(text, landscape)
    orient = 'landscape' if landscape else 'portrait'
    print(f"  Orientation: {orient}")

    with open(path, 'w') as f:
        f.write(text)

    print(f"  Updated {path}")

if __name__ == '__main__':
    for path in sys.argv[1:]:
        process_file(path)
