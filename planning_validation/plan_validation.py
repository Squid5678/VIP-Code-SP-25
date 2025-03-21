import subprocess
import os


'''This script will perform the following steps
1- Check if plan.txt exists. - This is the plan previously generated by an LLM and saved in a .txt file
2- Run VAL to verify the plan's correctness.
3- Display whether the plan achieves the goal or fails validation.
'''


# --------------- STEP 1: CHECK FILES EXIST --------------- #

def check_file_exists(file_path):
    """Checks if a file exists."""
    if not os.path.exists(file_path):
        print(f"Error: {file_path} not found.")
        return False
    return True

# Ensure required files exist
files = ["blocks_domain.pddl", "blocks_problem.pddl", "blocks_plan.txt"]
if not all(check_file_exists(f) for f in files):
    exit("Missing required PDDL files.")

print("All PDDL files found.")

# --------------- STEP 2: VALIDATE PDDL PLAN --------------- #

def validate_pddl_plan(domain_file, problem_file, plan_file):
    """Runs VAL to check if the plan is valid for the given domain and problem."""
    try:
        result = subprocess.run(
            ["val", domain_file, problem_file, plan_file], capture_output=True, text=True
        )

        if "successful plans:" in result.stdout:
            print("PDDL plan is valid and correctly achieves the goal.")
            return True
        else:
            print("PDDL plan validation failed.")
            print(result.stdout)
            return False

    except FileNotFoundError:
        print("VAL tool not installed. Please install it.")
        return False

# Run validation
if validate_pddl_plan("domain.pddl", "problem.pddl", "plan.txt"):
    print("Plan execution is successful.")
else:
    print("Plan execution failed. Check errors above.")