"""Copy results.txt into README.md between the results markers."""
from pathlib import Path

START, END = "<!-- results:start -->", "<!-- results:end -->"
body = Path("results.txt").read_text().strip()
readme = Path("README.md").read_text()
a, b = readme.index(START), readme.index(END) + len(END)
Path("README.md").write_text(readme[:a] + "\n".join([START, "```text", body, "```", END]) + readme[b:])
print("README.md results updated")
