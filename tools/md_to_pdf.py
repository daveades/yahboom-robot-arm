#!/usr/bin/env python3
"""Render a markdown file as a print-ready A4 document.

Output is one self-contained HTML file with no external assets, laid out for
paper rather than a screen: a cover page carrying the title and the opening
prose, the contents on their own page when the document has a contents
section, and a page break before each top-level heading.

    python3 tools/md_to_pdf.py --pdf                     # the user manual
    python3 tools/md_to_pdf.py --md README.md --pdf
    python3 tools/md_to_pdf.py --md notes.md --no-cover --break-before none

--break-before defaults to auto, which gives a page to each `#` heading, or
to each `##` heading when the document has fewer than three `#` headings.

--pdf drives a headless Chrome, including a Windows-side Chrome or Edge when
running under WSL. Without one, open the HTML in any browser and print it:
Ctrl+P, Destination "Save as PDF", Paper A4, Margins "Default", and turn
"Headers and footers" off so the page numbers in the CSS are the only ones.

Supported markdown: headings, fenced code, tables, nested and task lists,
blockquotes, and inline code, bold, italic and links. Images, footnotes,
reference links and raw HTML are not handled.
"""
import argparse
import html
import pathlib
import re
import shutil
import subprocess
import sys

# ---------- inline ----------

CODE_SPAN = re.compile(r'`([^`]+)`')
LINK = re.compile(r'\[([^\]]+)\]\(([^)]+)\)')
BOLD = re.compile(r'\*\*(.+?)\*\*')
ITALIC = re.compile(r'(?<!\*)\*([^*\s][^*]*)\*(?!\*)')


def inline(text):
    """Markdown inline -> HTML, with code spans protected from the rest."""
    spans = []

    def stash(m):
        spans.append(html.escape(m.group(1)))
        return "\x00%d\x00" % (len(spans) - 1)

    text = CODE_SPAN.sub(stash, text)
    text = html.escape(text)
    # escape() mangles the link/emphasis markers' neighbours, never the
    # markers themselves, so the patterns below still match.
    text = LINK.sub(lambda m: '<a href="%s">%s</a>' % (m.group(2), m.group(1)),
                    text)
    text = BOLD.sub(r'<strong>\1</strong>', text)
    text = ITALIC.sub(r'<em>\1</em>', text)
    return re.sub(r'\x00(\d+)\x00',
                  lambda m: '<code>%s</code>' % spans[int(m.group(1))], text)


def slug(text):
    """GitHub's heading-anchor algorithm, so existing §-links keep working."""
    return re.sub(r'[^a-z0-9 \-]', '', text.lower()).replace(' ', '-')


# ---------- blocks ----------

HEADING = re.compile(r'^(#{1,6}) (.+)$')
UL_ITEM = re.compile(r'^(\s*)([-*]) (.*)$')
OL_ITEM = re.compile(r'^(\s*)(\d+)\. (.*)$')
FENCE = re.compile(r'^```(\w*)\s*$')


def is_block_start(line):
    return bool(HEADING.match(line) or FENCE.match(line)
                or UL_ITEM.match(line) or OL_ITEM.match(line)
                or line.startswith('|') or line.startswith('>'))


def parse(lines):
    out, i, n = [], 0, len(lines)
    while i < n:
        line = lines[i]

        if not line.strip():
            i += 1
            continue

        m = FENCE.match(line)
        if m:
            lang, body, i = m.group(1), [], i + 1
            while i < n and not FENCE.match(lines[i]):
                body.append(lines[i])
                i += 1
            i += 1                                    # closing fence
            cls = ' class="lang-%s"' % lang if lang else ''
            out.append('<pre%s><code>%s</code></pre>'
                       % (cls, html.escape('\n'.join(body))))
            continue

        m = HEADING.match(line)
        if m:
            level, text = len(m.group(1)), m.group(2)
            out.append('<h%d id="%s">%s</h%d>'
                       % (level, slug(text), inline(text), level))
            i += 1
            continue

        if line.startswith('|'):
            rows, i = [], i
            while i < n and lines[i].startswith('|'):
                rows.append(lines[i])
                i += 1
            out.append(table(rows))
            continue

        if line.startswith('>'):
            body, i = [], i
            while i < n and lines[i].startswith('>'):
                body.append(re.sub(r'^> ?', '', lines[i]))
                i += 1
            out.append('<blockquote>%s</blockquote>' % parse(body))
            continue

        if UL_ITEM.match(line) or OL_ITEM.match(line):
            block, i = collect_list(lines, i)
            out.append(block)
            continue

        para, i = [], i
        while i < n and lines[i].strip() and not is_block_start(lines[i]):
            para.append(lines[i].strip())
            i += 1
        out.append('<p>%s</p>' % inline(' '.join(para)))
    return '\n'.join(out)


def split_row(row):
    row = row.strip().strip('|').replace('\\|', '\x01')
    return [c.strip().replace('\x01', '|') for c in row.split('|')]


def table(rows):
    if len(rows) < 2 or not re.match(r'^\|[\s:|-]+\|$', rows[1]):
        return '<p>%s</p>' % inline(' '.join(rows))
    head = split_row(rows[0])
    body = [split_row(r) for r in rows[2:]]
    out = ['<table><thead><tr>']
    out += ['<th>%s</th>' % inline(c) for c in head]
    out.append('</tr></thead><tbody>')
    for r in body:
        out.append('<tr>%s</tr>'
                   % ''.join('<td>%s</td>' % inline(c) for c in r))
    out.append('</tbody></table>')
    return ''.join(out)


def collect_list(lines, i):
    """Consume one whole list, returning (html, next_index).

    An item owns every following line that is blank or indented past its
    marker, which is what lets a nested list or a fenced code block sit
    inside a list item.
    """
    ordered = bool(OL_ITEM.match(lines[i]))
    pat = OL_ITEM if ordered else UL_ITEM
    indent = len(pat.match(lines[i]).group(1))
    items, n = [], len(lines)

    while i < n:
        m = pat.match(lines[i])
        if not m or len(m.group(1)) != indent:
            break
        body = [m.group(3)]
        i += 1
        while i < n:
            nxt = lines[i]
            if not nxt.strip():
                # a blank line only continues the item if indented text follows
                j = i + 1
                while j < n and not lines[j].strip():
                    j += 1
                if j < n and len(lines[j]) - len(lines[j].lstrip()) > indent:
                    body.append('')
                    i += 1
                    continue
                break
            if len(nxt) - len(nxt.lstrip()) > indent:
                body.append(nxt)
                i += 1
                continue
            break
        items.append(body)

    # dedent each item's continuation lines, then parse it as a document
    html_items = []
    for body in items:
        cont = [l for l in body[1:] if l.strip()]
        pad = min((len(l) - len(l.lstrip()) for l in cont), default=0)
        rest = [l[pad:] if l.strip() else '' for l in body[1:]]
        first = body[0]
        check = re.match(r'^\[([ xX])\] (.*)$', first)
        cls = ''
        if check:
            cls = ' class="task"'
            box = '☑' if check.group(1).lower() == 'x' else '☐'
            first = '%s %s' % (box, check.group(2))
        inner = parse([first] + rest)
        # A lone paragraph inside <li> reads better unwrapped. The lookahead
        # keeps this from swallowing a second paragraph and unbalancing the tags.
        one = re.fullmatch(r'<p>((?:(?!</p>).)*)</p>', inner, re.S)
        html_items.append('<li%s>%s</li>' % (cls, one.group(1) if one else inner))

    tag = 'ol' if ordered else 'ul'
    return '<%s>%s</%s>' % (tag, ''.join(html_items), tag), i


# ---------- page assembly ----------

CSS = """
@page {
  size: A4;
  margin: 20mm 18mm 18mm 18mm;
  @bottom-center { content: counter(page); font: 9pt Georgia, serif; }
}
@page :first { margin: 0; @bottom-center { content: none; } }

* { box-sizing: border-box; }
body {
  font: 10.5pt/1.5 Georgia, "DejaVu Serif", "Liberation Serif", serif;
  color: #000; margin: 0; hyphens: none;
}

/* ---- cover ---- */
.cover { break-after: page; height: 297mm; padding: 60mm 22mm 22mm; }
.cover h1 { font-size: 28pt; line-height: 1.15; margin: 0 0 6mm; border: 0; }
.cover .sub { font-size: 12pt; max-width: 130mm; line-height: 1.5; }
.cover .sub p { margin: 0 0 4mm; text-align: left; }
.cover .sub blockquote { margin: 5mm 0 0; font-size: 10.5pt; }

/* ---- contents ---- */
.contents { break-after: page; }
.contents h2 { border: 0; margin-top: 0; }

/* ---- headings ---- */
h1 {
  break-after: avoid; font-size: 18pt; margin: 0 0 6mm;
  padding-bottom: 2.5mm; border-bottom: 1pt solid #000;
}
.cover h1, .doctitle, .contents h2, body > :first-child {
  break-before: auto;
}
h2 { break-after: avoid; font-size: 13pt; margin: 7mm 0 2.5mm; }
h3 { break-after: avoid; font-size: 11pt; margin: 5mm 0 2mm;
     font-style: italic; }
h1 + h2, h2 + h3 { margin-top: 3mm; }

p, li { orphans: 3; widows: 3; }
p { margin: 0 0 2.6mm; text-align: justify; }
a { color: inherit; text-decoration: none; }

/* ---- lists ---- */
ul, ol { margin: 0 0 2.6mm; padding-left: 6.5mm; }
li { margin-bottom: 1.4mm; }
li > ul, li > ol { margin-top: 1.4mm; }
li.task { list-style: none; margin-left: -5mm; }

/* ---- code ---- */
pre {
  break-inside: avoid;
  font: 8pt/1.42 "DejaVu Sans Mono", "Liberation Mono", Consolas, monospace;
  border: 0.4pt solid #999; padding: 2.4mm 3mm; margin: 0 0 3mm;
  white-space: pre-wrap; overflow-wrap: break-word;
}
pre code { font: inherit; padding: 0; }
code {
  font: 0.88em "DejaVu Sans Mono", "Liberation Mono", Consolas, monospace;
  overflow-wrap: break-word;
}

/* ---- tables ---- */
table {
  width: 100%; border-collapse: collapse; margin: 0 0 3.5mm;
  font-size: 9pt; break-inside: auto;
}
thead { display: table-header-group; }
tr { break-inside: avoid; break-after: auto; }
th, td { border: 0.4pt solid #999; padding: 1.5mm 2mm;
         text-align: left; vertical-align: top; }
th { font-weight: bold; border-bottom-width: 0.8pt; }
.contents table { font-size: 10pt; }
.contents th:first-child, .contents td:first-child {
  width: 12mm; text-align: center; }

/* ---- callouts ---- */
blockquote {
  break-inside: avoid; margin: 0 0 3.5mm; padding: 0 0 0 4mm;
  border-left: 0.8pt solid #999;
}
blockquote > :last-child { margin-bottom: 0; }
"""


TOC_TITLE = re.compile(r'^#{1,3}\s+(table of )?contents\s*$', re.I)


def split_front(lines):
    """(title, lead, contents, body) for any document.

    The title is the first top-level heading, the lead is whatever prose
    follows it, and a "Contents" section, if the document has one, is kept
    aside for its own page. Any of the three may come back empty.
    """
    start = next((i for i, l in enumerate(lines) if l.startswith('# ')), None)
    if start is None:
        return '', [], [], lines                     # no title: all body

    title = lines[start][2:].strip()
    rest = lines[start + 1:]

    nxt = next((i for i, l in enumerate(rest) if l.startswith('#')), len(rest))
    lead, rest = rest[:nxt], rest[nxt:]

    contents = []
    if rest and TOC_TITLE.match(rest[0]):
        end = next((i for i, l in enumerate(rest[1:], 1) if l.startswith('#')),
                   len(rest))
        contents, rest = rest[:end], rest[end:]
    return title, lead, contents, rest


def pick_break_level(body):
    """Which heading level gets a page to itself.

    Breaking on a level used once, or not at all, would produce one enormous
    page, so take the shallowest level used at least three times.
    """
    for level in (1, 2):
        pat = re.compile(r'^#{%d} ' % level)
        if sum(1 for l in body if pat.match(l)) >= 3:
            return 'h%d' % level
    return 'none'


def build(md_path, out_path, cover=True, break_before='auto'):
    lines = pathlib.Path(md_path).read_text(encoding='utf-8').split('\n')
    title, lead, contents, body = split_front(lines)

    if break_before == 'auto':
        break_before = pick_break_level(body)
    rule = ('%s { break-before: page; }' % break_before
            if break_before != 'none' else '')

    parts = []
    if cover and title:
        parts.append('<section class="cover"><h1>%s</h1><div class="sub">%s'
                     '</div></section>' % (html.escape(title), parse(lead)))
    elif title:
        parts.append('<h1 class="doctitle">%s</h1>%s'
                     % (html.escape(title), parse(lead)))

    if contents:
        parts.append('<section class="contents">%s</section>' % parse(contents))
    parts.append(parse(body))

    doc = ('<!doctype html>\n<html lang="en"><head><meta charset="utf-8">\n'
           '<title>%s</title><style>%s\n%s</style></head><body>\n%s\n'
           '</body></html>\n'
           % (html.escape(title or pathlib.Path(md_path).stem), CSS, rule,
              '\n'.join(parts)))

    pathlib.Path(out_path).write_text(doc, encoding='utf-8')
    return doc


WIN_BROWSERS = [
    "/mnt/c/Program Files/Google/Chrome/Application/chrome.exe",
    "/mnt/c/Program Files (x86)/Google/Chrome/Application/chrome.exe",
    "/mnt/c/Program Files (x86)/Microsoft/Edge/Application/msedge.exe",
    "/mnt/c/Program Files/Microsoft/Edge/Application/msedge.exe",
]


def find_browser():
    for name in ('google-chrome', 'chromium', 'chromium-browser',
                 'microsoft-edge'):
        path = shutil.which(name)
        if path:
            return path, False
    for path in WIN_BROWSERS:
        if pathlib.Path(path).exists():
            return path, True
    return None, False


def to_pdf(html_path, pdf_path):
    """Render the HTML with headless Chrome. Returns True on success."""
    browser, windows = find_browser()
    if not browser:
        print("no Chrome/Edge found - open %s and print it to PDF yourself"
              % html_path)
        return False

    if windows:
        # The Windows binary cannot read Linux paths, so hand it UNC ones.
        def win(p):
            return subprocess.run(['wslpath', '-w', str(pathlib.Path(p).resolve())],
                                  capture_output=True, text=True,
                                  check=True).stdout.strip()
        src = 'file:///' + win(html_path).replace('\\', '/')
        pathlib.Path(pdf_path).touch()          # so wslpath can resolve it
        dst = win(pdf_path)
    else:
        src = pathlib.Path(html_path).resolve().as_uri()
        dst = str(pathlib.Path(pdf_path).resolve())

    subprocess.run([browser, '--headless', '--disable-gpu',
                    '--no-pdf-header-footer',
                    '--print-to-pdf=' + dst, src],
                   check=True, capture_output=True)
    return pathlib.Path(pdf_path).stat().st_size > 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--md', default='docs/user_manual.md',
                    help="markdown to render (default the user manual)")
    ap.add_argument('--out', help="HTML output (default: alongside --md)")
    ap.add_argument('--pdf', nargs='?', const=True,
                    help="also render a PDF (default: alongside --md)")
    ap.add_argument('--no-cover', action='store_true',
                    help="skip the cover page and run the title inline")
    ap.add_argument('--break-before', default='auto',
                    choices=('auto', 'h1', 'h2', 'none'),
                    help="heading level that starts a new page (default auto)")
    args = ap.parse_args()

    md = pathlib.Path(args.md)
    if not md.exists():
        sys.exit("%s not found - run this from the repo root." % args.md)
    out = args.out or str(md.with_suffix('.html'))
    doc = build(args.md, out, cover=not args.no_cover,
                break_before=args.break_before)
    print("wrote %s (%.0f KB)" % (out, len(doc) / 1024))

    if args.pdf:
        pdf = str(md.with_suffix('.pdf')) if args.pdf is True else args.pdf
        if to_pdf(out, pdf):
            print("wrote %s (%.0f KB)"
                  % (pdf, pathlib.Path(pdf).stat().st_size / 1024))
    return 0


if __name__ == '__main__':
    sys.exit(main())
