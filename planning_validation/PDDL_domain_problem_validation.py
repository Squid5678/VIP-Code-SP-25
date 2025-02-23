import subprocess
import os

# This code uses VAL PDDL validator to perform syntax and semantics checks on the domain and problem PDDL descriptions.

# --------------- STEP 1: READ EXISTING PDDL FILES --------------- #
# This part assumes that an LLM is previously used to generate and save PDDL versions of the domain and / 
# problem descriptions as text files name "domain.pddl" and "problem.pddl"

def read_pddl_file(file_path):
    """Reads a PDDL file and returns its content."""
    if os.path.exists(file_path):
        with open(file_path, "r") as file:
            return file.read()
    else:
        print(f"Error: {file_path} not found.")
        return None

# Load LLM-generated PDDL files
domain_pddl = read_pddl_file("blocks_domain.pddl")
problem_pddl = read_pddl_file("blocks_problem.pddl")

if domain_pddl is None or problem_pddl is None:
    exit("PDDL files are missing. Please check the file paths.")

print("PDDL files successfully loaded.")

# --------------- STEP 2: PDDL VALIDATION USING VAL --------------- #

def validate_pddl_syntax(domain_file, problem_file):
    """Runs VAL PDDL validator to check syntax."""
    try:
        result = subprocess.run(
            ["val", domain_file, problem_file], capture_output=True, text=True
        )
        if "Error" in result.stderr:
            print("Syntax Error in PDDL:")
            print(result.stderr)
            return False
        print("PDDL syntax is valid.")
        return True
    except FileNotFoundError:
        print("VAL tool not installed. Please install it.")
        return False

def validate_pddl_semantics(domain_file, problem_file):
    """Runs VAL to check logical validity."""
    try:
        result = subprocess.run(
            ["val", domain_file, problem_file], capture_output=True, text=True
        )
        if "Error" in result.stderr:
            print("Semantic Error in PDDL:")
            print(result.stderr)
            return False
        print("PDDL semantics are valid.")
        return True
    except FileNotFoundError:
        print("VAL tool not installed. Please install it.")
        return False

# Run validation
if validate_pddl_syntax("domain.pddl", "problem.pddl") and validate_pddl_semantics("domain.pddl", "problem.pddl"):
    print("PDDL files are valid and ready for planning.")
else:
    print("PDDL validation failed. Check the errors above.")
