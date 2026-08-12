#!/usr/bin/env python3
"""Per-section word count for paper.tex against the ICRA 8-page budget.

The budget column is measured from P-AgNav (RA-L 2025), which is exactly eight
IEEE two-column pages including references. It is a target, not a guess.

Deliberately does not depend on texcount: this runs before TeX is installed,
and it counts what a reader reads -- comments, math, and markup are stripped.

    python3 budget.py [paper.tex]
"""
import re
import sys

# section title (lowercase substring match) -> word budget
BUDGET = [
    ("abstract", 150),
    ("introduction", 1200),
    ("system overview", 990),
    ("system design", 2831),
    ("experimental results", 1000),
    ("conclusion", 250),
]
TOTAL_BUDGET = 7372  # includes ~958 w of references


def strip_latex(text):
    """Reduce LaTeX to roughly the prose a reader sees."""
    text = re.sub(r"(?<!\\)%.*", "", text)                 # comments
    text = re.sub(r"\$\$.*?\$\$|\$[^$]*\$", " ", text, flags=re.S)  # inline math
    for env in ("equation", "align", "table", "figure", "tabular"):
        text = re.sub(rf"\\begin\{{{env}\*?\}}.*?\\end\{{{env}\*?\}}",
                      " ", text, flags=re.S)
    text = re.sub(r"\\[a-zA-Z@]+\*?(\[[^\]]*\])?", " ", text)  # commands
    text = re.sub(r"[{}~^_&\\]", " ", text)
    return text


def sections(path):
    """Yield (title, wordcount) in document order, abstract first."""
    src = open(path).read()
    body = src.split(r"\begin{document}", 1)[-1]

    abstract = re.search(r"\\begin\{abstract\}(.*?)\\end\{abstract\}", body, re.S)
    if abstract:
        yield "Abstract", len(strip_latex(abstract.group(1)).split())
        body = body.replace(abstract.group(0), "")

    marks = list(re.finditer(r"\\section\{([^}]*)\}", body))
    for i, m in enumerate(marks):
        end = marks[i + 1].start() if i + 1 < len(marks) else len(body)
        chunk = body[m.end():end]
        chunk = re.sub(r"\\bibliography\{.*", "", chunk, flags=re.S)
        yield m.group(1), len(strip_latex(chunk).split())


def budget_for(title):
    low = title.lower()
    for key, n in BUDGET:
        if key in low:
            return n
    return None


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "paper.tex"
    total = 0
    over = False
    print(f"{'section':28s} {'words':>7s} {'budget':>7s}   status")
    print("-" * 60)
    for title, words in sections(path):
        total += words
        b = budget_for(title)
        if b is None:
            status = ""
        elif words > b:
            status = f"OVER by {words - b}"
            over = True
        else:
            status = f"{b - words} left"
        print(f"{title[:28]:28s} {words:7d} {b if b else '-':>7}   {status}")
    print("-" * 60)
    slack = TOTAL_BUDGET - 958 - total  # references not yet written
    print(f"{'BODY TOTAL':28s} {total:7d} {TOTAL_BUDGET - 958:7d}   "
          f"{'OVER by ' + str(-slack) if slack < 0 else str(slack) + ' left'}")
    print("\nPage count is the real limit -- `make pages` after TeX is installed.")
    return 1 if over or slack < 0 else 0


if __name__ == "__main__":
    sys.exit(main())
