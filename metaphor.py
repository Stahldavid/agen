import os
import json
from dotenv import load_dotenv
from metaphor_python import Metaphor

# Load environment variables from .env file
load_dotenv()

def metaphor_web_search(query: str, num_results: int = 10, include_domains: list = None, 
                        start_published_date: str = None, end_published_date: str = None, 
                        use_autoprompt: bool = False):
    """
    Searches the web using the Metaphor API based on the provided query and additional options.
    
    Parameters:
        query (str): The search query.
        num_results (int): The number of search results to return. Default is 10.
        include_domains (list): A list of domains to include in the search. Default is None.
        start_published_date (str): The start date for when the document was published (in YYYY-MM-DD format). Default is None.
        end_published_date (str): The end date for when the document was published (in YYYY-MM-DD format). Default is None.
        use_autoprompt (bool): Whether to use autoprompt for the search. Default is False.
        
    Returns:
        str: A JSON-formatted string containing the title and URL of each search result.
    """
    
    # Read API key from environment variable
    api_key = os.getenv("METAPHOR_API_KEY")
    
    client = Metaphor(api_key=api_key)
    response = client.search(
        query,
        num_results=num_results,
        include_domains=include_domains,
        start_published_date=start_published_date,
        end_published_date=end_published_date,
        use_autoprompt=use_autoprompt
    )
    
    results = [{"title": result.title, "url": result.url} for result in response.results]
    
    # Convert the list of dictionaries to a JSON-formatted string
    results_str = json.dumps(results)
    
    return results_str



# query = "The best ALGORITHM methods for variable impedance control for force feedback/haptics are:"
# num_results = 10
# use_autoprompt = False

# search_results_str = metaphor_web_search(query, num_results=num_results, use_autoprompt=use_autoprompt)
# print(search_results_str)
