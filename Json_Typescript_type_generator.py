#include <bits/stdc++.h>
using namespace std;

enum class JsonKind { Empty, Logical, Decimal, Text, List, Dict };

struct JsonNode {
    JsonKind kind = JsonKind::Empty;
    string text;
    double number = 0;
    bool flag = false;
    vector<JsonNode> list;
    vector<pair<string, JsonNode>> dict;
};

static string sourceJson;
static size_t cursorPos;

void consumeSpaces() {
    while (cursorPos < sourceJson.size() &&
          (sourceJson[cursorPos] == ' ' ||
           sourceJson[cursorPos] == '\t' ||
           sourceJson[cursorPos] == '\n' ||
           sourceJson[cursorPos] == '\r')) {
        cursorPos++;
    }
}

JsonNode readValue();

JsonNode readString() {
    cursorPos++;
    string result;

    while (cursorPos < sourceJson.size() && sourceJson[cursorPos] != '"') {
        if (sourceJson[cursorPos] == '\\') {
            cursorPos++;
            if (cursorPos < sourceJson.size()) {
                char ch = sourceJson[cursorPos++];
                if (ch == 'n') result += '\n';
                else if (ch == 't') result += '\t';
                else if (ch == 'r') result += '\r';
                else result += ch;
            }
        } else {
            result += sourceJson[cursorPos++];
        }
    }

    if (cursorPos < sourceJson.size()) cursorPos++;

    JsonNode node;
    node.kind = JsonKind::Text;
    node.text = result;
    return node;
}

JsonNode readValue() {
    consumeSpaces();

    if (cursorPos >= sourceJson.size()) return {};

    char current = sourceJson[cursorPos];

    if (current == '"') return readString();

    if (current == '{') {
        cursorPos++;

        JsonNode node;
        node.kind = JsonKind::Dict;

        while (true) {
            consumeSpaces();

            if (cursorPos < sourceJson.size() && sourceJson[cursorPos] == '}') {
                cursorPos++;
                break;
            }

            JsonNode keyNode = readString();

            consumeSpaces();
            if (cursorPos < sourceJson.size() && sourceJson[cursorPos] == ':') {
                cursorPos++;
            }

            JsonNode valueNode = readValue();
            node.dict.push_back({keyNode.text, valueNode});

            consumeSpaces();

            if (cursorPos < sourceJson.size() && sourceJson[cursorPos] == ',') {
                cursorPos++;
            } else if (cursorPos < sourceJson.size() && sourceJson[cursorPos] == '}') {
                cursorPos++;
                break;
            }
        }

        return node;
    }

    if (current == '[') {
        cursorPos++;

        JsonNode node;
        node.kind = JsonKind::List;

        while (true) {
            consumeSpaces();

            if (cursorPos < sourceJson.size() && sourceJson[cursorPos] == ']') {
                cursorPos++;
                break;
            }

            node.list.push_back(readValue());

            consumeSpaces();

            if (cursorPos < sourceJson.size() && sourceJson[cursorPos] == ',') {
                cursorPos++;
            } else if (cursorPos < sourceJson.size() && sourceJson[cursorPos] == ']') {
                cursorPos++;
                break;
            }
        }

        return node;
    }

    if (current == 't') {
        cursorPos += 4;
        JsonNode node;
        node.kind = JsonKind::Logical;
        node.flag = true;
        return node;
    }

    if (current == 'f') {
        cursorPos += 5;
        JsonNode node;
        node.kind = JsonKind::Logical;
        node.flag = false;
        return node;
    }

    if (current == 'n') {
        cursorPos += 4;
        return {};
    }

    size_t begin = cursorPos;

    if (cursorPos < sourceJson.size() && sourceJson[cursorPos] == '-') {
        cursorPos++;
    }

    while (cursorPos < sourceJson.size() &&
          (isdigit(sourceJson[cursorPos]) ||
           sourceJson[cursorPos] == '.' ||
           sourceJson[cursorPos] == 'e' ||
           sourceJson[cursorPos] == 'E' ||
           sourceJson[cursorPos] == '+' ||
           sourceJson[cursorPos] == '-')) {
        cursorPos++;
    }

    JsonNode node;
    node.kind = JsonKind::Decimal;
    node.number = stod(sourceJson.substr(begin, cursorPos - begin));
    return node;
}

struct FieldData {
    int seen = 0;
    bool objectFound = false;
    bool arrayFound = false;
    bool arrayObjectFound = false;
    set<string> primitiveTypes;
    set<string> arrayPrimitiveTypes;
};

struct ObjectData {
    int objectCount = 0;
    map<string, FieldData> fields;
};

using ObjectPath = vector<string>;

void scanObject(const JsonNode& node, const ObjectPath& currentPath, map<ObjectPath, ObjectData>& registry) {
    if (node.kind != JsonKind::Dict) return;

    ObjectData& data = registry[currentPath];
    data.objectCount++;

    for (auto& [fieldName, childNode] : node.dict) {
        FieldData& field = data.fields[fieldName];
        field.seen++;

        ObjectPath nextPath = currentPath;
        nextPath.push_back(fieldName);

        if (childNode.kind == JsonKind::Dict) {
            field.objectFound = true;
            scanObject(childNode, nextPath, registry);
        } else if (childNode.kind == JsonKind::List) {
            field.arrayFound = true;

            for (auto& item : childNode.list) {
                if (item.kind == JsonKind::Dict) {
                    field.arrayObjectFound = true;
                    scanObject(item, nextPath, registry);
                } else if (item.kind == JsonKind::Empty) {
                    field.arrayPrimitiveTypes.insert("null");
                } else if (item.kind == JsonKind::Logical) {
                    field.arrayPrimitiveTypes.insert("boolean");
                } else if (item.kind == JsonKind::Decimal) {
                    field.arrayPrimitiveTypes.insert("number");
                } else if (item.kind == JsonKind::Text) {
                    field.arrayPrimitiveTypes.insert("string");
                }
            }
        } else if (childNode.kind == JsonKind::Empty) {
            field.primitiveTypes.insert("null");
        } else if (childNode.kind == JsonKind::Logical) {
            field.primitiveTypes.insert("boolean");
        } else if (childNode.kind == JsonKind::Decimal) {
            field.primitiveTypes.insert("number");
        } else if (childNode.kind == JsonKind::Text) {
            field.primitiveTypes.insert("string");
        }
    }
}

void buildNames(const ObjectPath& currentPath,
                const map<ObjectPath, ObjectData>& registry,
                map<ObjectPath, string>& pathName,
                set<string>& reservedNames) {
    auto found = registry.find(currentPath);
    if (found == registry.end()) return;

    for (auto& [fieldName, field] : found->second.fields) {
        ObjectPath nextPath = currentPath;
        nextPath.push_back(fieldName);

        bool needsInterface = field.objectFound || field.arrayObjectFound;
        if (!needsInterface) continue;

        if (pathName.find(nextPath) == pathName.end()) {
            string baseName = fieldName;
            baseName[0] = toupper(baseName[0]);

            string finalName = baseName;
            int suffix = 2;

            while (reservedNames.count(finalName)) {
                finalName = baseName + to_string(suffix++);
            }

            pathName[nextPath] = finalName;
            reservedNames.insert(finalName);
        }

        buildNames(nextPath, registry, pathName, reservedNames);
    }
}

string makeType(const ObjectPath& currentPath,
                const string& fieldName,
                const FieldData& field,
                const map<ObjectPath, string>& pathName) {
    ObjectPath nextPath = currentPath;
    nextPath.push_back(fieldName);

    vector<string> choices;

    if (field.arrayFound) {
        vector<string> elementTypes;

        for (auto& typeName : field.arrayPrimitiveTypes) {
            elementTypes.push_back(typeName);
        }

        if (field.arrayObjectFound) {
            elementTypes.push_back(pathName.at(nextPath));
        }

        sort(elementTypes.begin(), elementTypes.end());

        string arrayType;

        if (elementTypes.empty()) {
            arrayType = "unknown[]";
        } else if (elementTypes.size() == 1) {
            arrayType = elementTypes[0] + "[]";
        } else {
            string combined = elementTypes[0];

            for (size_t i = 1; i < elementTypes.size(); i++) {
                combined += " | " + elementTypes[i];
            }

            arrayType = "(" + combined + ")[]";
        }

        choices.push_back(arrayType);
    }

    if (field.objectFound) {
        choices.push_back(pathName.at(nextPath));
    }

    for (auto& typeName : field.primitiveTypes) {
        choices.push_back(typeName);
    }

    sort(choices.begin(), choices.end());

    string result = choices[0];

    for (size_t i = 1; i < choices.size(); i++) {
        result += " | " + choices[i];
    }

    return result;
}

int main() {
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);

    int testCount;
    if (!(cin >> testCount)) return 0;

    string unusedLine;
    getline(cin, unusedLine);

    for (int caseNo = 0; caseNo < testCount; caseNo++) {
        if (caseNo > 0) cout << "---\n";

        string mainInterface;
        if (!getline(cin, mainInterface)) break;
        if (!mainInterface.empty() && mainInterface.back() == '\r') {
            mainInterface.pop_back();
        }

        string jsonInput;
        if (!getline(cin, jsonInput)) break;
        if (!jsonInput.empty() && jsonInput.back() == '\r') {
            jsonInput.pop_back();
        }

        sourceJson = jsonInput;
        cursorPos = 0;

        JsonNode rootNode = readValue();

        map<ObjectPath, ObjectData> registry;
        registry[{}].objectCount = 0;

        if (rootNode.kind == JsonKind::List) {
            for (auto& item : rootNode.list) {
                scanObject(item, {}, registry);
            }
        } else if (rootNode.kind == JsonKind::Dict) {
            scanObject(rootNode, {}, registry);
        }

        map<ObjectPath, string> pathName;
        set<string> reservedNames;

        pathName[{}] = mainInterface;
        reservedNames.insert(mainInterface);

        buildNames({}, registry, pathName, reservedNames);

        map<string, string> resultCode;

        for (auto& [path, interfaceName] : pathName) {
            auto found = registry.find(path);

            if (found == registry.end() || found->second.fields.empty()) {
                resultCode[interfaceName] = "export interface " + interfaceName + " {}";
                continue;
            }

            string code = "export interface " + interfaceName + " {\n";
            int totalObjects = found->second.objectCount;

            for (auto& [fieldName, field] : found->second.fields) {
                bool optional = field.seen < totalObjects;

                code += "  " + fieldName;
                if (optional) code += "?";
                code += ": " + makeType(path, fieldName, field, pathName) + ";\n";
            }

            code += "}";
            resultCode[interfaceName] = code;
        }

        bool firstOutput = true;

        for (auto& [interfaceName, code] : resultCode) {
            if (!firstOutput) cout << "\n\n";
            firstOutput = false;
            cout << code;
        }

        cout << "\n";
    }

    return 0;
}