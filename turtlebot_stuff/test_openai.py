from openai import OpenAI

client = OpenAI(api_key="NOPE")

# Set your OpenAI API key

# Function to read the prompt from a text file
def read_prompt_file(filename):
    try:
        with open(filename, 'r') as f:
            content = f.read()
        print(f"Successfully read prompt from '{filename}'.")
        return content
    except Exception as e:
        print(f"Could not read file '{filename}': {e}")
        return ""

# Function to send the prompt to OpenAI and parse the response
def get_move_parameters_from_openai(prompt_text):
    system_message = (
        "You are the brain of a turtlebot, and you are provided with a prompt that a human has given you. "
        "The prompt consists of a length in time the robot should move and direction (where direction is one of: 0=forward, 1=backward, 2=left, 3=right). "
        "Return them in the format: <duration> <direction>."
    )

    user_message = f"User's command: {prompt_text}\n\n"

    try:
        response = client.chat.completions.create(model="gpt-4o-mini",  # Use the latest GPT-4 model
        messages=[
            {"role": "system", "content": system_message},
            {"role": "user", "content": user_message}
        ],
        temperature=0.0)  # Deterministic output)
        content = response.choices[0].message.content.strip()
        print(f"OpenAI raw response: {content}")
        return content
    except Exception as e:
        print(f"Error calling OpenAI API: {e}")
        return None

# Main execution
if __name__ == '__main__':
    file_name = "command.txt"  # Replace with your text file name
    prompt_text = read_prompt_file(file_name)

    if prompt_text:
        openai_response = get_move_parameters_from_openai(prompt_text)
        if openai_response:
            print(f"OpenAI's reply: {openai_response}")
        else:
            print("Failed to get a valid response from OpenAI.")
    else:
        print("No prompt text to process.")