#include <bits/stdc++.h>
using namespace std;

enum class JT { Null, Bool, Num, Str, Arr, Obj };

struct JV {
  JT type = JT::Null;
  string s;
  double n = 0;
  bool b = false;
  vector<JV> arr;
  vector<pair<string, JV>> obj;
};

static string gs;
static size_t gp;

void skip() {
  while (gp < gs.size() &&
         (gs[gp] == ' ' || gs[gp] == '\t' || gs[gp] == '\n' || gs[gp] == '\r'))
    gp++;
}

JV parse_val();

JV parse_str_val() {
  gp++;
  string r;
  while (gp < gs.size() && gs[gp] != '"') {
    if (gs[gp] == '\\') {
      gp++;
      if (gp < gs.size()) {
        char c = gs[gp++];
        if (c == 'n')
          r += '\n';
        else if (c == 't')
          r += '\t';
        else if (c == 'r')
          r += '\r';
        else
          r += c;
      }
    } else {
      r += gs[gp++];
    }
  }
  if (gp < gs.size())
    gp++;
  JV v;
  v.type = JT::Str;
  v.s = r;
  return v;
}

JV parse_val() {
  skip();
  if (gp >= gs.size())
    return {};
  char c = gs[gp];
  if (c == '"')
    return parse_str_val();
  if (c == '{') {
    gp++;
    JV v;
    v.type = JT::Obj;
    while (true) {
      skip();
      if (gp < gs.size() && gs[gp] == '}') {
        gp++;
        break;
      }
      JV key = parse_str_val();
      skip();
      if (gp < gs.size() && gs[gp] == ':')
        gp++;
      JV val = parse_val();
      v.obj.push_back({key.s, val});
      skip();
      if (gp < gs.size() && gs[gp] == ',')
        gp++;
      else if (gp < gs.size() && gs[gp] == '}') {
        gp++;
        break;
      }
    }
    return v;
  }
  if (c == '[') {
    gp++;
    JV v;
    v.type = JT::Arr;
    while (true) {
      skip();
      if (gp < gs.size() && gs[gp] == ']') {
        gp++;
        break;
      }
      v.arr.push_back(parse_val());
      skip();
      if (gp < gs.size() && gs[gp] == ',')
        gp++;
      else if (gp < gs.size() && gs[gp] == ']') {
        gp++;
        break;
      }
    }
    return v;
  }
  if (c == 't') {
    gp += 4;
    JV v;
    v.type = JT::Bool;
    v.b = true;
    return v;
  }
  if (c == 'f') {
    gp += 5;
    JV v;
    v.type = JT::Bool;
    v.b = false;
    return v;
  }
  if (c == 'n') {
    gp += 4;
    return {};
  }
  size_t start = gp;
  if (gp < gs.size() && gs[gp] == '-')
    gp++;
  while (gp < gs.size() && (isdigit(gs[gp]) || gs[gp] == '.' || gs[gp] == 'e' ||
                            gs[gp] == 'E' || gs[gp] == '+' || gs[gp] == '-'))
    gp++;
  JV v;
  v.type = JT::Num;
  v.n = stod(gs.substr(start, gp - start));
  return v;
}

struct KInfo {
  int count = 0;
  bool has_obj = false;
  bool has_arr = false;
  bool arr_has_obj = false;
  set<string> prims;
  set<string> arr_prims;
};

struct PathInfo {
  int total = 0;
  map<string, KInfo> keys;
};

using Path = vector<string>;

void collect(const JV &val, const Path &path, map<Path, PathInfo> &infos) {
  if (val.type != JT::Obj)
    return;
  PathInfo &pi = infos[path];
  pi.total++;
  for (auto &[k, sv] : val.obj) {
    KInfo &ki = pi.keys[k];
    ki.count++;
    Path child = path;
    child.push_back(k);
    if (sv.type == JT::Obj) {
      ki.has_obj = true;
      collect(sv, child, infos);
    } else if (sv.type == JT::Arr) {
      ki.has_arr = true;
      for (auto &e : sv.arr) {
        if (e.type == JT::Obj) {
          ki.arr_has_obj = true;
          collect(e, child, infos);
        } else if (e.type == JT::Null)
          ki.arr_prims.insert("null");
        else if (e.type == JT::Bool)
          ki.arr_prims.insert("boolean");
        else if (e.type == JT::Num)
          ki.arr_prims.insert("number");
        else if (e.type == JT::Str)
          ki.arr_prims.insert("string");
      }
    } else if (sv.type == JT::Null)
      ki.prims.insert("null");
    else if (sv.type == JT::Bool)
      ki.prims.insert("boolean");
    else if (sv.type == JT::Num)
      ki.prims.insert("number");
    else if (sv.type == JT::Str)
      ki.prims.insert("string");
  }
}

void assign_names(const Path &path, const map<Path, PathInfo> &infos,
                  map<Path, string> &p2n, set<string> &used) {
  auto it = infos.find(path);
  if (it == infos.end())
    return;
  for (auto &[k, ki] : it->second.keys) {
    Path child = path;
    child.push_back(k);
    bool is_iface = ki.has_obj || ki.arr_has_obj;
    if (!is_iface)
      continue;
    if (p2n.find(child) == p2n.end()) {
      string base = k;
      base[0] = toupper(base[0]);
      string name = base;
      int suf = 2;
      while (used.count(name)) {
        name = base + to_string(suf++);
      }
      p2n[child] = name;
      used.insert(name);
    }
    assign_names(child, infos, p2n, used);
  }
}

string type_str(const Path &path, const string &k, const KInfo &ki,
                const map<Path, string> &p2n) {
  Path child = path;
  child.push_back(k);
  vector<string> parts;
  if (ki.has_arr) {
    vector<string> ep;
    for (auto &s : ki.arr_prims)
      ep.push_back(s);
    if (ki.arr_has_obj)
      ep.push_back(p2n.at(child));
    sort(ep.begin(), ep.end());
    string at;
    if (ep.empty())
      at = "unknown[]";
    else if (ep.size() == 1)
      at = ep[0] + "[]";
    else {
      string u = ep[0];
      for (size_t i = 1; i < ep.size(); i++)
        u += " | " + ep[i];
      at = "(" + u + ")[]";
    }
    parts.push_back(at);
  }
  if (ki.has_obj)
    parts.push_back(p2n.at(child));
  for (auto &s : ki.prims)
    parts.push_back(s);
  sort(parts.begin(), parts.end());
  string r = parts[0];
  for (size_t i = 1; i < parts.size(); i++)
    r += " | " + parts[i];
  return r;
}

int main() {
  ios_base::sync_with_stdio(false);
  cin.tie(NULL);

  int T;
  if (!(cin >> T))
    return 0;
  string dummy;
  getline(cin, dummy);

  for (int t = 0; t < T; t++) {
    if (t > 0)
      cout << "---\n";

    string root_name;
    if (!getline(cin, root_name))
      break;
    if (!root_name.empty() && root_name.back() == '\r')
      root_name.pop_back();

    string json_line;
    if (!getline(cin, json_line))
      break;
    if (!json_line.empty() && json_line.back() == '\r')
      json_line.pop_back();

    gs = json_line;
    gp = 0;
    JV root = parse_val();

    map<Path, PathInfo> infos;
    infos[{}].total = 0;

    if (root.type == JT::Arr) {
      for (auto &obj : root.arr)
        collect(obj, {}, infos);
    } else if (root.type == JT::Obj) {
      collect(root, {}, infos);
    }

    map<Path, string> p2n;
    set<string> used;
    p2n[{}] = root_name;
    used.insert(root_name);
    assign_names({}, infos, p2n, used);

    map<string, string> outputs;
    for (auto &[path, name] : p2n) {
      auto it = infos.find(path);
      if (it == infos.end() || it->second.keys.empty()) {
        outputs[name] = "export interface " + name + " {}";
        continue;
      }
      string code = "export interface " + name + " {\n";
      int total = it->second.total;
      for (auto &[k, ki] : it->second.keys) {
        bool opt = (ki.count < total);
        code += "  " + k + (opt ? "?" : "") + ": " +
                type_str(path, k, ki, p2n) + ";\n";
      }
      code += "}";
      outputs[name] = code;
    }

    bool first = true;
    for (auto &[name, code] : outputs) {
      if (!first)
        cout << "\n\n";
      first = false;
      cout << code;
    }
    cout << "\n";
  }
  return 0;
}