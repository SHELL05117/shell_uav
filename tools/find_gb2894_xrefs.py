#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Re-derive the (page index, xref) table that make_task_markers.py extracts from.

WHY THIS EXISTS
---------------
make_task_markers.py pulls the official GB 2894-2025 附录C 表C.1 warning
pictograms straight out of the standard with

    doc.extract_image(xref)

and an xref is a number with no meaning attached to it.  Get one wrong and you
do not get an error, you get a different triangle - and every warning sign in
this standard is a triangle, so the wrong answer looks entirely plausible until
somebody reads the symbol.  The numbers are also not stable: re-export the PDF
and they shift.  This script is how the table was produced in the first place,
so it can be reproduced rather than remembered.

HOW IT FINDS THEM
-----------------
Table C.1 is a grid: the number ("1-06") sits in a left-hand column and the
artwork in a column to its right, on the same row.  So the page gives the row
and the artwork column gives the column:

  1. the page index comes from the code, by the table's own pagination
     (page 1 holds 1-01..1-03, then four signs per page)
  2. page.get_text("dict") finds the span whose text is the code, and its
     y centre is the row
  3. page.get_image_info(xrefs=True) lists every embedded bitmap on the page;
     keep the ones whose x falls inside the artwork column and take the one
     whose y centre is nearest the row

Reading it rather than measuring it is deliberate.  Matching the extracted
artwork against the reference scans by correlation was tried and abandoned:
all fifty-two entries are the same black triangle on the same yellow field, so
full-image correlation is dominated by the shared outline and returns
whichever triangle happens to be least cropped, not the right one.  The row's
own y coordinate does not have that problem.

Usage
-----
    python3 tools/find_gb2894_xrefs.py                 # check the current table
    python3 tools/find_gb2894_xrefs.py --code 1-30     # probe one code
    python3 tools/find_gb2894_xrefs.py --pdf PATH
"""

import argparse
import glob
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

# The artwork column.  Table C.1 also has a text column on the left and the
# page has a header bitmap, so this band is what separates "the sign" from
# "everything else on the page".  Measured off the standard's own layout; the
# numbers are safe because the column is ~210 pt wide and the band is well
# inside it.
COL_X0, COL_X1 = 100.0, 330.0


def find_pdf(explicit):
    if explicit:
        if os.path.exists(explicit):
            return explicit
        sys.exit("--pdf %s not found" % explicit)
    home = os.path.expanduser("~")
    cands = [
        os.path.join(home, "Desktop", "页面提取自－GB+2894-2025.pdf"),
        os.path.join(home, "Downloads", "GB+2894-2025.pdf"),
    ]
    cands += sorted(glob.glob(os.path.join(home, "Desktop", "*2894-2025*.pdf")))
    cands += sorted(glob.glob(os.path.join(home, "Downloads", "*2894-2025*.pdf")))
    for c in cands:
        if os.path.exists(c):
            return c
    sys.exit("GB 2894-2025 PDF not found; pass --pdf PATH")


def page_index_for(code):
    """Appendix C's pagination: page 1 carries 1-01..1-03, then four per page."""
    number = int(code.split("-")[1])
    if number <= 3:
        return 1
    return 2 + (number - 4) // 4


def code_y_centre(page, code):
    """y centre of the span that reads `code`, or None."""
    for block in page.get_text("dict")["blocks"]:
        for line in block.get("lines", []):
            for span in line.get("spans", []):
                if span["text"].strip() == code:
                    y0, y1 = span["bbox"][1], span["bbox"][3]
                    return 0.5 * (y0 + y1)
    return None


def xref_on_row(page, code):
    """(xref, bbox) of the artwork sharing `code`'s row, or None."""
    y = code_y_centre(page, code)
    if y is None:
        return None
    best = None
    for info in page.get_image_info(xrefs=True):
        x0, y0, x1, y1 = info["bbox"]
        if not (COL_X0 <= 0.5 * (x0 + x1) <= COL_X1):
            continue
        gap = abs(0.5 * (y0 + y1) - y)
        if best is None or gap < best[0]:
            best = (gap, info["xref"], (round(x0), round(y0), round(x1), round(y1)))
    return None if best is None else (best[1], best[2])


def main():
    import make_task_markers as M

    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pdf", default=None)
    ap.add_argument("--code", action="append", default=[],
                    help="probe one code (repeatable); default is the SIGNS table")
    args = ap.parse_args()

    import fitz

    pdf = find_pdf(args.pdf)
    doc = fitz.open(pdf)

    wanted = args.code if args.code else [c for _n, _m, c, _cn, _x in M.SIGNS]
    declared = {c: x for _n, _m, c, _cn, x in M.SIGNS}

    print("source : %s  (%d pages)" % (pdf, doc.page_count))
    print()
    print("%-6s %-6s %-8s %-6s %-8s %s"
          % ("code", "page", "row", "xref", "declared", "bbox"))

    bad = 0
    for code in wanted:
        page_no = page_index_for(code)
        if not 0 <= page_no < doc.page_count:
            print("%-6s page %d is outside the document" % (code, page_no))
            bad += 1
            continue
        page = doc[page_no]
        found = xref_on_row(page, code)
        if found is None:
            print("%-6s %-6d %-8s %-6s %-8s no artwork on that row"
                  % (code, page_no, "-", "-", "-"))
            bad += 1
            continue
        xref, bbox = found
        expected = declared.get(code)
        note = ""
        if expected is not None and expected != (page_no, xref):
            note = "  <-- MISMATCH"
            bad += 1
        print("%-6s %-6d %-8s %-6d %-8s %s%s"
              % (code, page_no, "found", xref,
                 "%d,%d" % expected if expected else "-", bbox, note))

    print()
    if bad:
        print("%d entr%s could not be confirmed - do NOT trust the SIGNS table."
              % (bad, "y" if bad == 1 else "ies"))
        return 1
    print("all %d entries confirmed." % len(wanted))
    return 0


if __name__ == "__main__":
    sys.exit(main())
