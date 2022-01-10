# FRC Team 4069 Team Code
This repository is for the FRC 2022 season.

For communcation, please use the **#f1-programming_controls** channel on the team's Slack server.

### Downloads
For editing, download [VSCode] (https://code.visualstudio.com/download). In VSCode's extension store, download the [WPILib Extension] (https://marketplace.visualstudio.com/items?itemName=wpilibsuite.vscode-wpilib).

You may also find that [GitHub Desktop] (https://desktop.github.com/) is a useful tool to use while programming for the robot. GitHub Desktop will track changes to any files that you edit and allow you to commit them to the repository.

If you are using Linux, VSCode is supported natively. GitHub Desktop doesn't have native Linux support, but it can be installed from the command line using an open-source patch:
```
sudo wget https://github.com/shiftkey/desktop/releases/download/release-2.6.3-linux1/GitHubDesktop-linux-2.6.3-linux1.deb
sudo apt-get install gdebi-core
sudo gdebi GitHubDesktop-linux-2.6.3-linux1.deb
```

### Clean Code
For organization, please keep the code you write clean and readable.

- Use consistent indenations in your work. This just keeps function calls, variable calls, etc easier to read by making the methods/statements they are nested under more visible.

- Write comments explaining sections of code and what they do. This doesn't need to be excessive, but adding comments here and there will allow fellow team members to understand the code you have written better.

- Use the DRY (Don't Repeat Yourself) principle by moving repetitive blocks of code to functions. If you are using the same multi-line blocks of code over and over again, consider how a function can be created to automate this process for you.

- When creating functions, classes, etc, please create proper documentation by doing in-line JavaDocs. These are similar to block comments, but with an added asterisk on the first line and describe what the function does and its valid arguments. Example:
```
/**
* Checks if an int is greater than another int.
*
* @param arg0 First int.
* @param arg1 Second int.
*
* @return True if arg0 is higher than arg1.
*/
public boolean compare(int arg0, int arg1) {
    return arg0 > arg1;
}
