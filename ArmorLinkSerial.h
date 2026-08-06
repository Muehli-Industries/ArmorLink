#pragma once
#ifndef ARMORLINK_SERIAL_H
#define ARMORLINK_SERIAL_H

#include "ArmorLinkDebug.h"
#include <Arduino.h>
#include <functional>
#include <utility>
#include <vector>

#ifndef ARMORLINK_SERIAL_COMMAND_MAX_LEN
#define ARMORLINK_SERIAL_COMMAND_MAX_LEN 31
#endif

#ifndef ARMORLINK_SERIAL_LINE_MAX_LEN
#define ARMORLINK_SERIAL_LINE_MAX_LEN 1024
#endif

static constexpr const char* ARMORLINK_FLASHER_PREFIX = "@ALF:";

using ArmorLinkSerialCommandHandler =
    std::function<void(const String& args)>;

using ArmorLinkInternalSerialHandler =
    bool (*)(void* context, const String& payload);

using ArmorLinkUnknownSerialHandler =
    bool (*)(void* context, const String& line);

struct ArmorLinkSerialCommandDef {
    String name;
    ArmorLinkSerialCommandHandler handler;
};

class ArmorLinkSerialRegistry {
public:
    bool addCommand(
        const String& name,
        ArmorLinkSerialCommandHandler handler
    ) {
        String normalized = normalize(name);

        if (!isValidUserCommandName(normalized)) {
            Serial.printf(
                "[SERIAL] Invalid or reserved command name: %s\n",
                name.c_str()
            );
            return false;
        }

        if (!handler) {
            Serial.printf(
                "[SERIAL] Command has no handler: %s\n",
                normalized.c_str()
            );
            return false;
        }

        if (contains(normalized)) {
            Serial.printf(
                "[SERIAL] Command already registered: %s\n",
                normalized.c_str()
            );
            return false;
        }

        ArmorLinkSerialCommandDef command;
        command.name = normalized;
        command.handler = handler;

        _commands.push_back(command);
        return true;
    }

    template <typename Callable>
    bool addCommand(const char* name, Callable&& handler) {
        return addCommand(
            String(name ? name : ""),
            ArmorLinkSerialCommandHandler(
                std::forward<Callable>(handler)
            )
        );
    }

    bool removeCommand(const String& name) {
        const String normalized = normalize(name);

        for (auto it = _commands.begin();
             it != _commands.end();
             ++it) {
            if (it->name == normalized) {
                _commands.erase(it);
                return true;
            }
        }

        return false;
    }

    bool contains(const String& name) const {
        const String normalized = normalize(name);

        for (const auto& command : _commands) {
            if (command.name == normalized) {
                return true;
            }
        }

        return false;
    }

    bool dispatch(
        const String& commandName,
        const String& args
    ) const {
        const String normalized = normalize(commandName);

        for (const auto& command : _commands) {
            if (command.name == normalized) {
                command.handler(args);
                return true;
            }
        }

        return false;
    }

    const std::vector<ArmorLinkSerialCommandDef>& items() const {
        return _commands;
    }

    static bool isReserved(const String& text) {
        String normalized = text;
        normalized.trim();
        normalized.toLowerCase();

        return normalized.startsWith("@alf:") ||
               normalized.startsWith("alf.");
    }

private:
    std::vector<ArmorLinkSerialCommandDef> _commands;

    static String normalize(const String& value) {
        String normalized = value;
        normalized.trim();
        normalized.toLowerCase();
        return normalized;
    }

    static bool isValidUserCommandName(const String& name) {
        if (name.isEmpty() ||
            name.length() > ARMORLINK_SERIAL_COMMAND_MAX_LEN) {
            return false;
        }

        if (isReserved(name)) {
            return false;
        }

        for (size_t i = 0; i < name.length(); ++i) {
            const char c = name.charAt(i);

            const bool valid =
                (c >= 'a' && c <= 'z') ||
                (c >= '0' && c <= '9') ||
                c == '.' ||
                c == '_' ||
                c == '-';

            if (!valid) {
                return false;
            }
        }

        return true;
    }
};

class ArmorLinkSerialRuntime {
public:
    void begin(
        Stream& stream,
        ArmorLinkSerialRegistry& registry
    ) {
        _stream = &stream;
        _registry = &registry;
    }

    void setInternalHandler(
        ArmorLinkInternalSerialHandler handler,
        void* context = nullptr
    ) {
        _internalHandler = handler;
        _internalContext = context;
    }

    void setUnknownHandler(
        ArmorLinkUnknownSerialHandler handler,
        void* context = nullptr
    ) {
        _unknownHandler = handler;
        _unknownContext = context;
    }

    void loop() {
        if (_stream == nullptr || _registry == nullptr) {
            return;
        }

        while (_stream->available() > 0) {
            const char c =
                static_cast<char>(_stream->read());

            if (c == '\r') {
                continue;
            }

            if (c == '\n') {
                String line = _line;
                _line = "";

                line.trim();

                if (!line.isEmpty()) {
                    dispatchLine(line);
                }

                continue;
            }

            if (_line.length() <
                ARMORLINK_SERIAL_LINE_MAX_LEN) {
                _line += c;
            } else {
                _line = "";
                Serial.println(
                    "[SERIAL] Input line exceeded maximum length"
                );
            }
        }
    }

    bool dispatchLine(const String& input) {
        String line = input;
        line.trim();

        if (line.startsWith(ARMORLINK_FLASHER_PREFIX)) {
            const String payload = line.substring(
                strlen(ARMORLINK_FLASHER_PREFIX)
            );

            return _internalHandler
                ? _internalHandler(_internalContext, payload)
                : false;
        }

        const int separator =
            findFirstWhitespace(line);

        const String command =
            separator < 0
                ? line
                : line.substring(0, separator);

        String args =
            separator < 0
                ? ""
                : line.substring(separator + 1);

        args.trim();

        if (_registry->dispatch(command, args)) {
            return true;
        }

        return _unknownHandler
            ? _unknownHandler(_unknownContext, line)
            : false;
    }

private:
    Stream* _stream = nullptr;
    ArmorLinkSerialRegistry* _registry = nullptr;

    ArmorLinkInternalSerialHandler _internalHandler = nullptr;
    void* _internalContext = nullptr;
    ArmorLinkUnknownSerialHandler _unknownHandler = nullptr;
    void* _unknownContext = nullptr;

    String _line;

    static int findFirstWhitespace(
        const String& line
    ) {
        for (size_t i = 0; i < line.length(); ++i) {
            const char c = line.charAt(i);

            if (c == ' ' || c == '\t') {
                return static_cast<int>(i);
            }
        }

        return -1;
    }
};

#endif
