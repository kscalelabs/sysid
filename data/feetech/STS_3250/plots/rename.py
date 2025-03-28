import os
import re

# Change this to your target directory
directory = "."

for filename in os.listdir(directory):
    if filename.endswith(".pdf"):
        # Match pattern like: sts3250-12v_m1_ID004.pdf or sts3250-12v_m1_detail_ID004.pdf
        match = re.match(r"(.*)_ID(\d+)\.pdf", filename)
        if match:
            base, id_number = match.groups()
            # Replace underscores with dashes in base
            base = base.replace('_', '-')
            new_name = f"ID{id_number}-{base}.pdf"
            old_path = os.path.join(directory, filename)
            new_path = os.path.join(directory, new_name)
            print(f"Renaming: {filename} → {new_name}")
            os.rename(old_path, new_path)
