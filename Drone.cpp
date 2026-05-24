#include <algorithm>
#include <cmath>
#include <ctime>
#include <functional>
#include <iostream>
#include <json/json.h>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

int main() {
  // Read all input from stdin
  string input_str((istreambuf_iterator<char>(cin)),
                   istreambuf_iterator<char>());
  Json::Value input_data;
  Json::CharReaderBuilder rb;
  string errs;
  istringstream ss(input_str);
  Json::parseFromStream(rb, ss, &input_data, &errs);

  double mapW = input_data["map_size"][0].asDouble();
  double mapH = input_data["map_size"][1].asDouble();
  double warehouseX = mapW / 2.0, warehouseY = mapH / 2.0;
  Json::Value drones = input_data["drones"];
  Json::Value deliveries = input_data["deliveries"];
  Json::Value no_fly_zones =
      input_data.get("no_fly_zones", Json::Value(Json::arrayValue));
  Json::Value charging_stations =
      input_data.get("charging_stations", Json::Value(Json::arrayValue));
  // End of HEAD

  // Start of BODY
  /*
   * Schedule drone deliveries to maximize on-time deliveries
   * while minimizing energy and makespan.
   *
   * Input:
   *   warehouse: [x, y] - center of map, pickup/return location
   *   drones: array of {"id": str, "max_payload": double}
   *   deliveries: array of {"id": str, "x": double, "y": double, "weight":
   * double, "deadline": double} no_fly_zones: array of {"shape":
   * "circle"|"rectangle", "center"/"corners", "radius", "T_start", "T_end"}
   *   charging_stations: array of {"x": double, "y": double}
   *
   * Output:
   *   JSON: {"flight_manifest": [drone_entries]}
   *   Each drone_entry: {"drone_id": str, "path": [steps]}
   *   Each step: {"x": double, "y": double, "t": double, "action": str, ...}
   *   Actions: PICKUP (+delivery_ids), DELIVER (+delivery_id), RETURN, CHARGE,
   * CHARGE_COMPLETE, WAIT, WAYPOINT
   *
   * Scoring:
   *   score = (on_time_deliveries * 100) - (total_energy * 0.1) - (makespan *
   * 0.05) energy per leg = distance * (1 + current_payload_weight) Battery
   * capacity = 500, recharges on RETURN to warehouse
   */

  Json::Value flight_manifest(Json::arrayValue);

  const double BATTERY = 500.0;
  const double EPS = 1e-7;
  const clock_t START_CLOCK = clock();
  auto out_of_time = [&]() -> bool {
    return (double)(clock() - START_CLOCK) / CLOCKS_PER_SEC > 0.70;
  };

  struct Pt {
    double x, y;
  };

  struct DeliveryInfo {
    string id;
    Pt p;
    double weight;
    double deadline;
  };

  struct StationInfo {
    Pt p;
    int slots;
  };

  struct NFZInfo {
    string shape;
    Pt c;
    double r;
    double xmin, ymin, xmax, ymax;
    double ts, te;
  };

  struct Step {
    Pt p;
    double t;
    string action;
    vector<string> pickup_ids;
    string delivery_id;
  };

  struct ChargeUse {
    int station_index;
    double start;
    double finish;
  };

  struct Interval {
    bool ok;
    double l, r;
  };

  struct Trial {
    bool ok;
    vector<Step> steps;
    vector<ChargeUse> charges;
    double finish;
    double cost;
    Trial() : ok(false), finish(0), cost(0) {}
  };

  auto distance_between = [](Pt a, Pt b) -> double {
    return hypot(a.x - b.x, a.y - b.y);
  };

  auto energy_for = [](double d, double payload) -> double {
    return d * (1.0 + payload);
  };

  vector<DeliveryInfo> all_deliveries;
  for (Json::ArrayIndex i = 0; i < deliveries.size(); i++) {
    DeliveryInfo d;
    d.id = deliveries[i]["id"].asString();
    d.p = Pt{deliveries[i]["x"].asDouble(), deliveries[i]["y"].asDouble()};
    d.weight = deliveries[i]["weight"].asDouble();
    d.deadline = deliveries[i]["deadline"].asDouble();
    all_deliveries.push_back(d);
  }

  vector<StationInfo> stations;
  for (Json::ArrayIndex i = 0; i < charging_stations.size(); i++) {
    StationInfo st;
    st.p = Pt{charging_stations[i]["x"].asDouble(),
              charging_stations[i]["y"].asDouble()};
    st.slots = charging_stations[i].get("slots", 1).asInt();
    stations.push_back(st);
  }

  vector<vector<pair<double, double>>> station_reservations(stations.size());

  vector<NFZInfo> zones;
  for (Json::ArrayIndex i = 0; i < no_fly_zones.size(); i++) {
    NFZInfo z;
    z.shape = no_fly_zones[i]["shape"].asString();
    z.ts = no_fly_zones[i]["T_start"].asDouble();
    z.te = no_fly_zones[i]["T_end"].asDouble();
    z.r = z.xmin = z.ymin = z.xmax = z.ymax = 0.0;

    if (z.shape == "circle") {
      z.c = Pt{no_fly_zones[i]["center"][0].asDouble(),
               no_fly_zones[i]["center"][1].asDouble()};
      z.r = no_fly_zones[i]["radius"].asDouble();
    } else {
      z.xmin = no_fly_zones[i]["corners"][0][0].asDouble();
      z.ymin = no_fly_zones[i]["corners"][0][1].asDouble();
      z.xmax = no_fly_zones[i]["corners"][1][0].asDouble();
      z.ymax = no_fly_zones[i]["corners"][1][1].asDouble();
      if (z.xmin > z.xmax)
        swap(z.xmin, z.xmax);
      if (z.ymin > z.ymax)
        swap(z.ymin, z.ymax);
    }
    zones.push_back(z);
  }

  function<Interval(Pt, Pt, const NFZInfo &)> circle_interval =
      [&](Pt a, Pt b, const NFZInfo &z) -> Interval {
    double dx = b.x - a.x, dy = b.y - a.y;
    double fx = a.x - z.c.x, fy = a.y - z.c.y;
    double A = dx * dx + dy * dy;

    if (A < EPS) {
      bool inside = hypot(fx, fy) <= z.r + EPS;
      return Interval{inside, 0.0, 1.0};
    }

    double B = 2.0 * (fx * dx + fy * dy);
    double C = fx * fx + fy * fy - z.r * z.r;
    double D = B * B - 4.0 * A * C;
    if (D < -EPS)
      return Interval{false, 0.0, 0.0};

    D = max(0.0, D);
    double q = sqrt(D);
    double u1 = (-B - q) / (2.0 * A);
    double u2 = (-B + q) / (2.0 * A);
    double lo = max(0.0, min(u1, u2));
    double hi = min(1.0, max(u1, u2));

    return Interval{lo <= hi + EPS, lo, hi};
  };

  function<Interval(Pt, Pt, const NFZInfo &)> rect_interval =
      [&](Pt a, Pt b, const NFZInfo &z) -> Interval {
    double dx = b.x - a.x, dy = b.y - a.y;
    double lo = 0.0, hi = 1.0;

    double pvals[4] = {-dx, dx, -dy, dy};
    double qvals[4] = {a.x - z.xmin, z.xmax - a.x, a.y - z.ymin, z.ymax - a.y};

    for (int i = 0; i < 4; i++) {
      if (fabs(pvals[i]) < EPS) {
        if (qvals[i] < -EPS)
          return Interval{false, 0.0, 0.0};
      } else {
        double u = qvals[i] / pvals[i];
        if (pvals[i] < 0)
          lo = max(lo, u);
        else
          hi = min(hi, u);
        if (lo > hi + EPS)
          return Interval{false, 0.0, 0.0};
      }
    }
    return Interval{true, max(0.0, lo), min(1.0, hi)};
  };

  auto path_conflicts = [&](Pt a, Pt b, double depart,
                            double &wait_to) -> bool {
    bool blocked = false;
    wait_to = depart;
    double d = distance_between(a, b);

    for (const NFZInfo &z : zones) {
      Interval inside = (z.shape == "circle") ? circle_interval(a, b, z)
                                              : rect_interval(a, b, z);

      if (!inside.ok)
        continue;

      double enter_time = depart + inside.l * d;
      double exit_time = depart + inside.r * d;

      if (enter_time <= z.te + EPS && exit_time >= z.ts - EPS) {
        blocked = true;
        wait_to = max(wait_to, z.te + 1e-5);
      }
    }
    return blocked;
  };

  auto segment_touches_zone = [&](Pt from, Pt to, const NFZInfo &z) -> bool {
    Interval inside = (z.shape == "circle") ? circle_interval(from, to, z)
                                            : rect_interval(from, to, z);
    return inside.ok;
  };

  auto route_candidates = [&](Pt from, Pt to,
                              double depart) -> vector<vector<Pt>> {
    vector<vector<Pt>> routes;
    routes.push_back(vector<Pt>());

    double wait_probe = depart;
    if (!path_conflicts(from, to, depart, wait_probe))
      return routes;

    for (const NFZInfo &z : zones) {
      if (!segment_touches_zone(from, to, z))
        continue;

      if (z.shape == "circle") {
        double dx = to.x - from.x, dy = to.y - from.y;
        double len = hypot(dx, dy);
        if (len < EPS)
          continue;
        double nx = -dy / len, ny = dx / len;
        double rr = z.r + 5.0;
        routes.push_back(vector<Pt>{Pt{z.c.x + nx * rr, z.c.y + ny * rr}});
        routes.push_back(vector<Pt>{Pt{z.c.x - nx * rr, z.c.y - ny * rr}});
      } else {
        double m = 5.0;
        Pt bl{z.xmin - m, z.ymin - m};
        Pt br{z.xmax + m, z.ymin - m};
        Pt tr{z.xmax + m, z.ymax + m};
        Pt tl{z.xmin - m, z.ymax + m};

        routes.push_back(vector<Pt>{bl});
        routes.push_back(vector<Pt>{br});
        routes.push_back(vector<Pt>{tr});
        routes.push_back(vector<Pt>{tl});
        routes.push_back(vector<Pt>{bl, br});
        routes.push_back(vector<Pt>{br, tr});
        routes.push_back(vector<Pt>{tr, tl});
        routes.push_back(vector<Pt>{tl, bl});
      }

      if ((int)routes.size() >= 10)
        break;
    }

    return routes;
  };

  auto build_route = [&](Pt from, Pt to, double start_t,
                         const vector<Pt> &waypoints, vector<Step> &produced,
                         double &finish_t) -> bool {
    produced.clear();
    finish_t = start_t;
    Pt cur = from;
    vector<Pt> targets = waypoints;
    targets.push_back(to);

    for (int i = 0; i < (int)targets.size(); i++) {
      Pt target = targets[i];
      int guard = 0;

      while (true) {
        double wait_to = finish_t;
        if (!path_conflicts(cur, target, finish_t, wait_to))
          break;

        if (++guard > 200)
          return false;

        if (wait_to > finish_t + EPS) {
          finish_t = wait_to;
          Step wait_step;
          wait_step.p = cur;
          wait_step.t = finish_t;
          wait_step.action = "WAIT";
          produced.push_back(wait_step);
        } else {
          finish_t += 1e-5;
        }
      }

      finish_t += distance_between(cur, target);

      Step move_step;
      move_step.p = target;
      move_step.t = finish_t;
      move_step.action = (i + 1 == (int)targets.size()) ? "MOVE" : "WAYPOINT";
      produced.push_back(move_step);
      cur = target;
    }

    return true;
  };

  auto safe_move = [&](Pt from, Pt to, double &t) -> vector<Step> {
    vector<Step> best_steps;
    double best_finish = 1e100;
    double best_distance = 1e100;

    vector<vector<Pt>> candidates = route_candidates(from, to, t);
    for (const vector<Pt> &wps : candidates) {
      vector<Step> produced;
      double finish_t = t;
      if (!build_route(from, to, t, wps, produced, finish_t))
        continue;

      double route_dist = 0.0;
      Pt cur = from;
      for (Pt wp : wps) {
        route_dist += distance_between(cur, wp);
        cur = wp;
      }
      route_dist += distance_between(cur, to);

      if (finish_t < best_finish - 1e-6 ||
          (fabs(finish_t - best_finish) <= 1e-6 &&
           route_dist < best_distance)) {
        best_finish = finish_t;
        best_distance = route_dist;
        best_steps = produced;
      }
    }

    if (best_steps.empty()) {
      build_route(from, to, t, vector<Pt>(), best_steps, best_finish);
    }

    t = best_finish;
    return best_steps;
  };

  auto earliest_charge_start = [&](int station_index, double wanted,
                                   double duration) -> double {
    if (duration <= EPS)
      return wanted;
    int slots = max(1, stations[station_index].slots);
    double start = wanted;

    while (true) {
      int used = 0;
      double next_free = 1e100;

      for (auto interval : station_reservations[station_index]) {
        double a = interval.first;
        double b = interval.second;
        if (a < start + duration - EPS && b > start + EPS) {
          used++;
          next_free = min(next_free, b);
        }
      }

      if (used < slots)
        return start;
      if (next_free > 9e99)
        return start;
      start = next_free;
    }
  };

  function<Trial(const vector<const DeliveryInfo *> &, double)> simulate_trip =
      [&](const vector<const DeliveryInfo *> &batch,
          double start_time) -> Trial {
    Trial trial;
    if (out_of_time())
      return trial;
    if (batch.empty())
      return trial;

    Pt warehouse{warehouseX, warehouseY};
    double payload = 0.0;
    for (const DeliveryInfo *d : batch)
      payload += d->weight;

    double t = start_time;
    double battery = BATTERY;
    Pt cur = warehouse;

    Step pickup;
    pickup.p = warehouse;
    pickup.t = t;
    pickup.action = "PICKUP";
    for (const DeliveryInfo *d : batch)
      pickup.pickup_ids.push_back(d->id);
    trial.steps.push_back(pickup);

    auto raw_energy = [&](Pt start, const vector<Step> &raw,
                          double load) -> double {
      double e = 0.0;
      Pt prev = start;
      for (Step s : raw) {
        if (s.action != "WAIT") {
          e += energy_for(distance_between(prev, s.p), load);
          prev = s.p;
        }
      }
      return e;
    };

    auto commit_raw = [&](Pt start, const vector<Step> &raw, double load,
                          const string &final_action,
                          const string &delivery_id) -> bool {
      Pt prev = start;

      for (Step s : raw) {
        if (s.action == "WAIT") {
          trial.steps.push_back(s);
        } else if (s.action == "WAYPOINT") {
          double e = energy_for(distance_between(prev, s.p), load);
          battery -= e;
          trial.cost += e;
          if (battery < -1e-6)
            return false;
          trial.steps.push_back(s);
          prev = s.p;
        } else {
          double e = energy_for(distance_between(prev, s.p), load);
          battery -= e;
          trial.cost += e;
          if (battery < -1e-6)
            return false;

          Step out;
          out.p = s.p;
          out.t = s.t;
          out.action = final_action;
          out.delivery_id = delivery_id;
          trial.steps.push_back(out);

          prev = s.p;
        }
      }

      return true;
    };

    auto charge_here = [&](int station_index, Pt station,
                           double needed_after_charge) -> bool {
      double desired = min(BATTERY, max(needed_after_charge, battery));
      double add = max(0.0, desired - battery);
      double charge_time = ceil(add / 2.0);

      double charge_start =
          earliest_charge_start(station_index, t, charge_time);
      if (charge_start > t + EPS) {
        t = charge_start;
        Step wait_step;
        wait_step.p = station;
        wait_step.t = t;
        wait_step.action = "WAIT";
        trial.steps.push_back(wait_step);
      }

      Step charge;
      charge.p = station;
      charge.t = t;
      charge.action = "CHARGE";
      trial.steps.push_back(charge);

      t += charge_time;
      battery = min(BATTERY, battery + charge_time * 2.0);

      Step complete;
      complete.p = station;
      complete.t = t;
      complete.action = "CHARGE_COMPLETE";
      trial.steps.push_back(complete);

      if (charge_time > EPS) {
        trial.charges.push_back(ChargeUse{station_index, charge_start, t});
      }
      return battery + 1e-6 >= needed_after_charge;
    };

    auto move_with_charge = [&](Pt target, double load, const string &action,
                                const string &delivery_id) -> bool {
      double direct_t = t;
      vector<Step> direct_raw = safe_move(cur, target, direct_t);
      double direct_e = raw_energy(cur, direct_raw, load);

      if (direct_e <= battery + 1e-6) {
        t = direct_t;
        Pt start = cur;
        if (!commit_raw(start, direct_raw, load, action, delivery_id))
          return false;
        cur = target;
        return true;
      }

      int best_station = -1;
      double best_finish = 1e100;
      vector<Step> best_to_station, best_to_target;
      double best_charge_start = 0.0, best_charge_finish = 0.0;
      double best_e1 = 0.0, best_e2 = 0.0;

      for (int si = 0; si < (int)stations.size(); si++) {
        Pt station = stations[si].p;
        double t1 = t;
        vector<Step> raw1 = safe_move(cur, station, t1);
        double e1 = raw_energy(cur, raw1, load);
        if (e1 > battery + 1e-6)
          continue;

        double after_arrival_battery = battery - e1;
        double full_charge_time =
            ceil(max(0.0, BATTERY - after_arrival_battery) / 2.0);
        double cs = earliest_charge_start(si, t1, full_charge_time);
        double depart = cs + full_charge_time;

        double t2 = depart;
        vector<Step> raw2 = safe_move(station, target, t2);
        double e2 = raw_energy(station, raw2, load);
        if (e2 > BATTERY + 1e-6)
          continue;

        if (t2 < best_finish) {
          best_finish = t2;
          best_station = si;
          best_to_station = raw1;
          best_to_target = raw2;
          best_charge_start = cs;
          best_charge_finish = depart;
          best_e1 = e1;
          best_e2 = e2;
        }
      }

      if (best_station == -1)
        return false;

      Pt station = stations[best_station].p;
      double t_station = t;
      for (Step s : best_to_station)
        t_station = s.t;

      Pt start = cur;
      if (!commit_raw(start, best_to_station, load, "WAYPOINT", ""))
        return false;
      cur = station;
      t = t_station;

      if (best_charge_start > t + EPS) {
        t = best_charge_start;
        Step wait_step;
        wait_step.p = station;
        wait_step.t = t;
        wait_step.action = "WAIT";
        trial.steps.push_back(wait_step);
      }

      Step charge;
      charge.p = station;
      charge.t = t;
      charge.action = "CHARGE";
      trial.steps.push_back(charge);

      t = best_charge_finish;
      battery = BATTERY;

      Step complete;
      complete.p = station;
      complete.t = t;
      complete.action = "CHARGE_COMPLETE";
      trial.steps.push_back(complete);
      if (best_charge_finish > best_charge_start + EPS) {
        trial.charges.push_back(
            ChargeUse{best_station, best_charge_start, best_charge_finish});
      }

      if (!commit_raw(station, best_to_target, load, action, delivery_id))
        return false;
      cur = target;
      t = best_finish;
      return true;
    };

    double carrying = payload;
    for (const DeliveryInfo *d : batch) {
      if (out_of_time())
        return trial;
      if (!move_with_charge(d->p, carrying, "DELIVER", d->id))
        return trial;
      if (t > d->deadline + 1e-6)
        return trial;
      carrying -= d->weight;
    }

    if (!move_with_charge(warehouse, 0.0, "RETURN", ""))
      return trial;

    trial.ok = true;
    trial.finish = t;
    return trial;
  };

  auto order_batch =
      [&](vector<const DeliveryInfo *> batch) -> vector<const DeliveryInfo *> {
    vector<const DeliveryInfo *> ordered;
    Pt cur{warehouseX, warehouseY};

    while (!batch.empty()) {
      int best = 0;
      for (int i = 1; i < (int)batch.size(); i++) {
        double si = batch[i]->deadline +
                    0.02 * distance_between(cur, batch[i]->p) -
                    5.0 * batch[i]->weight;
        double sb = batch[best]->deadline +
                    0.02 * distance_between(cur, batch[best]->p) -
                    5.0 * batch[best]->weight;
        if (si < sb)
          best = i;
      }

      ordered.push_back(batch[best]);
      cur = batch[best]->p;
      batch.erase(batch.begin() + best);
    }

    return ordered;
  };

  vector<int> delivery_order(all_deliveries.size());
  iota(delivery_order.begin(), delivery_order.end(), 0);
  sort(delivery_order.begin(), delivery_order.end(), [&](int a, int b) {
    return all_deliveries[a].deadline < all_deliveries[b].deadline;
  });

  vector<vector<Step>> planned_paths(drones.size());
  vector<double> drone_time(drones.size(), 0.0);
  vector<int> done(all_deliveries.size(), 0);
  int batch_limit = (all_deliveries.size() > 80 || zones.size() > 8) ? 1 : 3;

  while (!out_of_time()) {
    int progress = 0;

    for (Json::ArrayIndex di = 0; di < drones.size(); di++) {
      if (out_of_time())
        break;
      double max_payload = drones[di]["max_payload"].asDouble();
      vector<const DeliveryInfo *> batch;
      double weight_sum = 0.0;

      for (int idx : delivery_order) {
        if (out_of_time())
          break;
        if (done[idx])
          continue;
        if ((int)batch.size() >= batch_limit)
          break;

        const DeliveryInfo *d = &all_deliveries[idx];
        if (d->weight > max_payload + 1e-9)
          continue;
        if (weight_sum + d->weight > max_payload + 1e-9)
          continue;

        vector<const DeliveryInfo *> candidate = batch;
        candidate.push_back(d);
        candidate = order_batch(candidate);

        Trial trial = simulate_trip(candidate, drone_time[di]);
        if (trial.ok) {
          batch = candidate;
          weight_sum += d->weight;
        }
      }

      if (batch.empty())
        continue;

      Trial trial = simulate_trip(batch, drone_time[di]);
      if (!trial.ok)
        continue;

      for (const DeliveryInfo *d : batch) {
        for (int i = 0; i < (int)all_deliveries.size(); i++) {
          if (&all_deliveries[i] == d)
            done[i] = 1;
        }
      }

      drone_time[di] = trial.finish;
      planned_paths[di].insert(planned_paths[di].end(), trial.steps.begin(),
                               trial.steps.end());
      for (const ChargeUse &use : trial.charges) {
        station_reservations[use.station_index].push_back(
            make_pair(use.start, use.finish));
      }
      progress++;
    }

    if (progress == 0)
      break;
  }

  for (Json::ArrayIndex di = 0; di < drones.size(); di++) {
    if (planned_paths[di].empty())
      continue;

    Json::Value drone_entry;
    drone_entry["drone_id"] = drones[di]["id"].asString();
    drone_entry["path"] = Json::Value(Json::arrayValue);

    for (const Step &s : planned_paths[di]) {
      Json::Value item;
      item["x"] = s.p.x;
      item["y"] = s.p.y;
      item["t"] = s.t;
      item["action"] = s.action;

      if (!s.pickup_ids.empty()) {
        item["delivery_ids"] = Json::Value(Json::arrayValue);
        for (const string &id : s.pickup_ids)
          item["delivery_ids"].append(id);
      }

      if (!s.delivery_id.empty()) {
        item["delivery_id"] = s.delivery_id;
      }

      drone_entry["path"].append(item);
    }

    flight_manifest.append(drone_entry);
  }

  // End of BODY

  // Start of TAIL
  Json::Value output;
  output["flight_manifest"] = flight_manifest;
  Json::StreamWriterBuilder wb;
  wb["indentation"] = "";
  cout << Json::writeString(wb, output) << endl;
  return 0;
}
// End of TAIL