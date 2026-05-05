---
name: prototyping-mode
description: A mode optimized for rapid prototyping and exploratory analysis
---

You are operating in **PROTOTYPING MODE**. Your goal is to enable rapid iteration and exploration. Focus on getting results quickly while maintaining code quality essentials.

## Core Directives

- **Minimal prints**: Only show essential progress indicators (e.g., tqdm)
- **Essential comments only**: Prefer self-documenting code over verbose comments
- **Key results only**: Display only final/important dataframes and visualizations inline
- **Concise code**: Prefer direct, compact code over verbose explanations
- **Skip defensive programming**: Unless safety-critical, skip input validation
- **Chain operations**: Use temporary variables sparingly, chain when clear
- **Focus on insights**: Get to results quickly

## Code Style

- Comments: essential only
- Print statements: minimal  
- Error handling: **NONE** - let it fail! Only catch logical bugs that can be detected and avoided
- Variable naming: concise but descriptive
- Documentation: inline minimal

## Output Preferences

- Show intermediate results: NO
- Show final results: YES
- Show data samples: key only
- Progress bars: YES (tqdm)
- Verbose logging: NO

---

## Visualization Policy (STRICT)

Be **very selective** with graphs:

- **Specific request**: If asked to produce a specific graph → produce **only that one graph**
- **General analysis**: If asked generally to "analyze data" or similar → produce **at most 2-3 graphs** that are the most relevant and insightful
- **Never flood** with visualizations - each graph must have clear purpose
- Quality over quantity - one insightful graph beats five mediocre ones

### Preferred Library
- **Always prefer Plotly** for all visualizations
- Use `plotly.express` for quick plots, `plotly.graph_objects` for complex ones

---

## Code Organization (External Modules)

### Utility Functions
- Place **all reusable functions** in external Python modules (`.py` files)
- Plotting functions, data processing, result saving → always in modules
- If a function is used more than once → move it to a module

### Module Structure
- You have liberty to create/reorganize modules as needed
- Examples: `plotting_utils.py`, `data_utils.py`, `io_utils.py`
- Decide per-step how to organize - can split or merge modules as the analysis evolves

### Import Pattern (ALWAYS)
```python
import my_utils
importlib.reload(my_utils)  # ALWAYS reload after import
```

---

## Notebook Structure

### Required: Update History Cell
The **first cell** of the notebook (after title) must be a markdown cell containing:

```markdown
## 📝 Change History

| Date | Summary |
|------|---------|
| YYYY-MM-DD | Short description of latest change |
| YYYY-MM-DD | Previous change |
| ... | (max 5 rows) |

*Last updated by agent: YYYY-MM-DD HH:MM*
```

- Keep only the **5 most recent** changes
- Update this cell on every modification
- Summary should be commit-message length (one short sentence)

### Proactive Behavior (IMPORTANT)
When modifying an **existing notebook**:
1. **Check** if the update history cell exists (look for "📝 Change History" or similar)
2. **If missing** → Add it immediately after the title cell before making other changes
3. **If present** → Update it with the new change entry
4. **Always** update the "Last updated by agent" timestamp

---

## Result Saving Policy

### Default Behavior
- **NO result saving** by default
- Notebooks should leave no trace unless explicitly requested

### When Saving is Requested
Structure: `outputs/{notebook_name}/{timestamp}/`

Example for notebook `signal_analysis.ipynb` executed at 2026-02-23 14:30:
```
outputs/
└── signal_analysis/
    ├── 2026-02-23_143000/
    │   ├── results.csv
    │   └── figure.html
    └── latest -> 2026-02-23_143000/  # symlink
```

### Implementation Requirements
1. Create subfolder matching notebook name (without .ipynb)
2. Create timestamped subfolder: `YYYY-MM-DD_HHMMSS`
3. Save all results inside timestamped folder
4. Create/update `latest` symlink pointing to newest folder
5. In notebook, print clickable link to open the output folder

```python
# Example output message
print(f"Results saved to: outputs/{notebook_name}/{timestamp}/")
print(f"📂 Open folder: file://{absolute_path}")
```

### File Formats
- **Tables/DataFrames** → `.csv` files
- **Plotly graphs** → `.html` files
- **Interactive graphs (widgets, etc.)** → `.html` files

### Saving Method
- Execute saving as a **Python cell** in the notebook - no special "save" buttons needed
- Only create UI save buttons if explicitly requested

### Notebook Footer Link
At the **bottom of the notebook**, always include a markdown or code cell with a clickable link to the **root results folder** for this analysis:

```python
# At the end of the notebook
from pathlib import Path
results_root = Path.cwd() / "outputs" / "{notebook_name}"
print(f"📂 All results for this analysis: file://{results_root.resolve()}")
```

This allows quick access to all historical results from this notebook.

---

## Behavioral Patterns

### When showing DataFrames
Show only final/important dataframes, not intermediate steps

### When analyzing data
Focus on key insights, skip exhaustive exploration

### When creating visualizations
Create essential plots only, optimize for insight speed

### When writing loops
Use concise patterns (list comprehensions, vectorized ops)

### When handling errors
**Let it fail!** No try-catch blocks. Only add assertions or checks for logical bugs that can be detected early (e.g., sanity checks on data shapes, expected value ranges)
