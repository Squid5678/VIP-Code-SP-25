### **Benchmarking & Validation for LLM Planning**

Three major components are essential for planning:

1. **domain.pddl**  
2. **problem.pddl**  
3. **plan.txt**

Since both PDDL and planning are highly specialized tasks, I think the benchmarking and validating domain descriptions, problem descriptions, and plans require highly specialized tools rather than generic metrics.

1. #### **Basic Syntax Checking – `pddlpy`**

   Various planning tools are available for parsing and syntax checking PDDL files. A simple Python library for structure validation is:  
* **`pddlpy`**: Useful for verifying PDDL structure but does not validate logic or consistency.

2. #### **Full Validation (domain, problem & plan) – VAL Tool**

   For comprehensive validation, including syntax, logic consistency, and plan success checks, the VAL tool seems to be a common option. It runs on Linux and requires downloading and compiling from GitHub:  
    [https://github.com/KCL-Planning/VAL](https://github.com/KCL-Planning/VAL)  
   
   For now, I set up an Ubuntu WSL environment on my Windows machine and successfully tested a simple block-placing task (see the files in the following folder with link) for plan validity via the command line. However, I haven’t yet tested the Python validation script, as it requires setting up an environment in Ubuntu. (I plan to do this in about 3 weeks once I get my other laptop back from repair.)  
   
   The validation scripts using the VAL tool and the sample files are stored in the github repo:  📂 ...VIP-Code-SP-25/planning\_validation  
   
   There are additional available planning tools that are similar to the VAL tool, all run on Ubuntu. I can also look into those as well in the future.

3. #### **Additional Tool – Online Planner**

   To generate simple plans, I also experimented with this online tool: [https://editor.planning.domains/\#](https://editor.planning.domains/#)

   However, only some of the available planning tools function correctly on the website.