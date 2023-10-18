import git
from typing import List, Optional
import os

def git_operations(
        operation_type: str, 
        repo_path: Optional[str] = None, 
        branch_name: Optional[str] = None, 
        commit_message: Optional[str] = None, 
        files: Optional[List[str]] = None,
        remote_name: Optional[str] = None,
        remote_url: Optional[str] = None,
        tag_name: Optional[str] = None,
        commit_sha: Optional[str] = None
    ) -> str:
    """
    Handle different git operations like clone, checkout, commit, push, pull, etc.

    :param operation_type: The type of git operation ('clone', 'checkout', 'commit', 'push', 'pull', 'init', 'status', 'log', 'add_remote', 'remove_remote', 'fetch', 'reset', 'tag').
    :param repo_path: The path to the git repository where the operation will be performed.
    :param branch_name: The name of the branch for checkout, commit, push, and pull operations.
    :param commit_message: The commit message for the commit operation.
    :param files: The list of file paths to add before committing.
    :param remote_name: The name of the remote for push, pull, add_remote, and remove_remote operations.
    :param remote_url: The URL of the remote repository for clone and remote add operations.
    :param tag_name: The name of the tag for the tag operation.
    :param commit_sha: The commit SHA for the reset operation.

    :returns: A string indicating the result of the git operation.
    """

        # If repo_path is not provided, use the current working directory
    if repo_path is None:
        repo_path = os.getcwd()
    try:
        if operation_type == 'clone':
            if remote_url is None:
                return "Remote URL is required for cloning."
            git.Repo.clone_from(remote_url, repo_path)
            return f"Repository cloned from {remote_url} to {repo_path}."

        if operation_type == 'init':
            git.Repo.init(repo_path)
            return f"Initialized empty Git repository in {repo_path}."

        repo = git.Repo(repo_path)

        if operation_type == 'checkout':
            if branch_name is None:
                return "Branch name is required for checkout."
            repo.git.checkout(branch_name)
            return f"Switched to branch {branch_name}."

        if operation_type == 'commit':
            if branch_name is None or commit_message is None or files is None:
                return "Branch name, commit message, and files are required for commit."
            repo.git.checkout(branch_name)
            repo.index.add(files)
            repo.index.commit(commit_message)
            return f"Committed changes to branch {branch_name} with message '{commit_message}'."

        if operation_type == 'push':
            if branch_name is None or remote_name is None:
                return "Branch name and remote name are required for push."
            origin = repo.remote(name=remote_name)
            origin.push(branch_name)
            return f"Pushed changes to remote {remote_name} on branch {branch_name}."

        if operation_type == 'pull':
            if branch_name is None or remote_name is None:
                return "Branch name and remote name are required for pull."
            origin = repo.remote(name=remote_name)
            origin.pull(branch_name)
            return f"Pulled latest changes from remote {remote_name} to branch {branch_name}."

        if operation_type == 'status':
            return str(repo.git.status())

        if operation_type == 'log':
            return str(repo.git.log('--oneline', '-5'))  # Last 5 commits

        if operation_type == 'add_remote':
            if remote_name is None or remote_url is None:
                return "Remote name and remote URL are required for add_remote."
            repo.create_remote(remote_name, url=remote_url)
            return f"Remote {remote_name} added with URL {remote_url}."

        if operation_type == 'remove_remote':
            if remote_name is None:
                return "Remote name is required for remove_remote."
            repo.delete_remote(remote_name)
            return f"Remote {remote_name} has been removed."

        if operation_type == 'fetch':
            if remote_name is None:
                return "Remote name is required for fetch."
            repo.remote(name=remote_name).fetch()
            return f"Fetched updates from {remote_name}."

        if operation_type == 'reset':
            if commit_sha is None:
                return "Commit SHA is required for reset."
            repo.git.reset('--hard', commit_sha)
            return f"HEAD is now at {commit_sha}"

        if operation_type == 'tag':
            if tag_name is None:
                return "Tag name is required for tag operation."
            repo.create_tag(tag_name)
            return f"Created tag {tag_name}."

        return "Invalid operation type."
    except git.GitCommandError as e:
        return f"Git operation failed: {str(e)}"

