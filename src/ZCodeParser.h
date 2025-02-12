#include <Arduino.h>
#include <vector>
#include <Shared/ZCommand.h>

class ZCodeParser
{
private:
    static void splitString(const char *input, char delimiter, std::vector<String> &result);

public:
    static std::vector<ZCommand> ParseString(const String &inString);
};

void ZCodeParser::splitString(const char *input, char delimiter, std::vector<String> &result)
{
    // Serial.println("Splitting String: "); 

    result.clear();
    size_t start = 0;
    size_t len = strlen(input);

    for (size_t i = 0; i <= len; i++)
    {
        if (input[i] == delimiter || input[i] == '\0')
        {
            if (i > start)
            {
                result.push_back(String(input + start).substring(0, i - start)); // Fix applied here
            }
            start = i + 1;
        }
    }

    // Serial.println("Split Result:");
    // for (const auto &s : result)
    // {
    //     Serial.print("[" + s + "] ");
    // }
    // Serial.println();
}

std::vector<ZCommand> ZCodeParser::ParseString(const String &inString)
{
    // Serial.println("\n--- Parsing Started ---");
    // Serial.println("Heap Before Parsing: " + String(ESP.getFreeHeap()));

    std::vector<String> stringCommands;
    splitString(inString.c_str(), '\n', stringCommands);

    // Serial.println("After Splitting into Commands:");
    // for (size_t i = 0; i < stringCommands.size(); i++)
    // {
    //     Serial.println("Command[" + String(i) + "]: " + stringCommands[i]);
    // }
    
    // Remove comments
    for (String &command : stringCommands)
    {
        int commentIndex = command.indexOf(';');
        if (commentIndex != -1)
        {
            command = command.substring(0, commentIndex);
            // Serial.println("Comment Removed: " + command);
        }
    }

    std::vector<ZCommand> commands;
    for (const String &commandLine : stringCommands)
    {
        std::vector<String> separatedStrings;
        splitString(commandLine.c_str(), ' ', separatedStrings);

        if (separatedStrings.empty())
            continue;

        ZCommand command;
        command.command = separatedStrings[0];

        // Serial.println("Processing Command: " + command.command);
        for (size_t j = 1; j < separatedStrings.size(); j++)
        {
            command.params.push_back(separatedStrings[j]);
            // Serial.println("Added Param: " + command.params.back());
        }

        commands.push_back(command);
        // Serial.println("Stored Command: " + commands.back().command);
    }

    // Serial.println("Heap After Parsing: " + String(ESP.getFreeHeap()));
    // Serial.println("--- Parsing Finished ---\n");

    return commands;
}
