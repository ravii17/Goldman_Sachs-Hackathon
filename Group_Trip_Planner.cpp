#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <map>
#include <set>
#include <algorithm>

using namespace std;

struct User {
    string name;
    int budget, energy;
    set<string> interests;
    bool active = true;
};

struct Activity {
    int id;
    string name, tag;
    int cost, duration, energy;
};

struct PlanDay {
    vector<int> ids;
    int cost = 0, sat = 0;
};

string formatDay(int day, const PlanDay& p) {
    if (p.ids.empty()) return "Day " + to_string(day) + ": REST | cost=0 satisfaction=0";
    string s = "Day " + to_string(day) + ":";
    vector<int> sorted_ids = p.ids;
    sort(sorted_ids.begin(), sorted_ids.end());
    for (int id : sorted_ids) s += " " + to_string(id);
    s += " | cost=" + to_string(p.cost) + " satisfaction=" + to_string(p.sat);
    return s;
}

PlanDay findBest(int H, const vector<User>& users, const vector<Activity>& eligible) {
    int min_b = 2000000000, min_e = 2000000000;
    bool any_active = false;
    for (const auto& u : users)
        if (u.active) { any_active = true; min_b = min(min_b, u.budget); min_e = min(min_e, u.energy); }
    if (!any_active) return {};

    PlanDay best;
    int n = eligible.size();
    for (int i = 0; i < (1 << n); ++i) {
        PlanDay cur;
        int cur_dur = 0, cur_e = 0;
        for (int j = 0; j < n; ++j) {
            if ((i >> j) & 1) {
                cur.ids.push_back(eligible[j].id);
                cur.cost += eligible[j].cost;
                cur_dur += eligible[j].duration;
                cur_e += eligible[j].energy;
                for (const auto& u : users)
                    if (u.active && u.interests.count(eligible[j].tag)) cur.sat++;
            }
        }
        if (cur_dur <= H && cur.cost <= min_b && cur_e <= min_e) {
            sort(cur.ids.begin(), cur.ids.end());
            if (cur.sat > best.sat) best = cur;
            else if (cur.sat == best.sat) {
                if (cur.cost < best.cost) best = cur;
                else if (cur.cost == best.cost && (best.ids.empty() || cur.ids < best.ids)) best = cur;
            }
        }
    }
    return best;
}

int N, D, H;
vector<User> users;
map<int, Activity> activities;
vector<PlanDay> current_plan;
map<int, set<string>> blocked_tags;

void run_sim(int start_day) {
    if (start_day < 1) start_day = 1;
    if (start_day > D) return;

    set<int> used;
    for (int d = 1; d < start_day; ++d)
        for (int id : current_plan[d].ids) used.insert(id);

    for (int d = start_day; d <= D; ++d) {
        vector<Activity> eligible;
        for (auto const& [id, act] : activities)
            if (!used.count(id) && !blocked_tags[d].count(act.tag))
                eligible.push_back(act);

        current_plan[d] = findBest(H, users, eligible);
        for (int id : current_plan[d].ids) used.insert(id);
        cout << formatDay(d, current_plan[d]) << "\n";
    }
}

void solve() {
    if (!(cin >> N >> D >> H)) return;

    users.resize(N);
    map<string, int> name_to_idx;
    for (int i = 0; i < N; ++i) {
        int k;
        cin >> users[i].name >> users[i].budget >> users[i].energy >> k;
        name_to_idx[users[i].name] = i;
        for (int j = 0; j < k; ++j) { string t; cin >> t; users[i].interests.insert(t); }
    }

    int A; cin >> A;
    for (int i = 0; i < A; ++i) {
        Activity act;
        cin >> act.id >> act.name >> act.cost >> act.duration >> act.energy >> act.tag;
        activities[act.id] = act;
    }

    int E; cin >> E; cin.ignore();
    vector<string> event_lines(E);
    for (int i = 0; i < E; ++i) getline(cin, event_lines[i]);

    current_plan.resize(D + 2);

    cout << "=== PLAN ===\n";
    run_sim(1);

    for (int i = 0; i < E; ++i) {
        stringstream ss(event_lines[i]);
        string type; ss >> type;
        int day; ss >> day;

        cout << "=== EVENT " << i + 1 << ": " << event_lines[i] << " ===\n";

        if (day < 1 || day > D) {
            continue;
        }

        if (type == "WEATHER") {
            string tag; ss >> tag;
            blocked_tags[day].insert(tag);
        } else if (type == "DROP") {
            string name; ss >> name;
            if (name_to_idx.count(name))
                users[name_to_idx[name]].active = false;
        } else if (type == "FATIGUE") {
            string name; int val; ss >> name >> val;
            if (name_to_idx.count(name))
                users[name_to_idx[name]].energy = val;
        } else if (type == "BUDGET") {
            string name; int val; ss >> name >> val;
            if (name_to_idx.count(name))
                users[name_to_idx[name]].budget = val;
        }

        run_sim(day);
    }
}

int main() {
    ios_base::sync_with_stdio(false);
    cin.tie(NULL);
    solve();
    return 0;
}