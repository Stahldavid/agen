import tiktoken

# Get the CL100KBase encoding and create a Tokenizer instance
cl100k_base = tiktoken.get_encoding("cl100k_base")
tokenizer = cl100k_base

def num_tokens_from_messages(messages):
    """
Calculates the number of tokens used so far in a conversation.

This uses the CL100KBase tokenizer to encode the conversation messages
and tally up the total number of tokens.

Each message is encoded as:
<im_start>{role/name}\n{content}<im_end>\n

Parameters:
    messages (list): The list of conversation messages. 
        Each message is a dict with "role" and "content" keys.
        
Returns:
    num_tokens (int): The total number of tokens used in the conversation.
"""
    encoding = tiktoken.get_encoding("cl100k_base")  # Model to encoding mapping
    num_tokens = 0
    for message in messages:
        num_tokens += 4  # Every message follows <im_start>{role/name}\n{content}<im_end>\n
        for key, value in message.items():
            if isinstance(value, str):  # Ensure value is a string before encoding
                num_tokens += len(encoding.encode(value))
                if key == "name":  # If there's a name, the role is omitted
                    num_tokens -= 1  # Role is always required and always 1 token
    num_tokens += 2  # Every reply is primed with <im_start>assistant
    return num_tokens

