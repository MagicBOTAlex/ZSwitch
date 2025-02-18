#ifndef ZCOMMAND_H
#define ZCOMMAND_H
#include <Arduino.h>
#include <vector>

typedef struct
{
    String command;
    std::vector<String> params;
} ZCommand;

#endif