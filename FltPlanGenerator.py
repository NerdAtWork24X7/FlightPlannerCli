import math
import sys
import os
import re
import pandas as pd
import networkx as nx
from sqlalchemy import create_engine
from lxml import etree

# ==========================
# CONFIG (defaults; overridable via env/CLI)
# ==========================
#DB_PATH = "Database\little_navmap.sqlite"
OUTPUT_FILE = "KJFK_WSSS_IFR.pln"

MAX_DISTANCE_KM = 1500       # bounding box size
CRUISE_FL = 33000
MAX_EDGE_KM = 600

# Environment prefs
PREFERRED_SID_NAME = os.getenv('SID_NAME') or "RAXE2A"
PREFERRED_SID_RUNWAY = os.getenv('SID_RWY') or "27"    
PREFERRED_STAR_NAME = os.getenv('STAR_NAME') or "DENU3A"
ATC_ROUTE = os.getenv('ATC_ROUTE')





# ==========================
# Utilities
# ==========================

class Utils:
    @staticmethod
    def haversine(lat1, lon1, lat2, lon2):
        R = 6371
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        dphi = math.radians(lat2 - lat1)
        dl = math.radians(lon2 - lon1)
        a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dl/2)**2
        return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))

    @staticmethod
    def bearing_deg(lat1, lon1, lat2, lon2):
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        dlon = math.radians(lon2 - lon1)
        y = math.sin(dlon) * math.cos(phi2)
        x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlon)
        br = math.degrees(math.atan2(y, x))
        return (br + 360) % 360

    @staticmethod
    def bounding_box(lat, lon, km):
        d = km / 111
        return lat-d, lat+d, lon-d, lon+d

    @staticmethod
    def bounding_box_route(lat1, lon1, lat2, lon2, margin_km):
        minlat = min(lat1, lat2) - margin_km / 111
        maxlat = max(lat1, lat2) + margin_km / 111
        avg_lat = math.radians((minlat + maxlat) / 2)
        margin_deg_lon = margin_km / (111 * math.cos(avg_lat))
        minlon = min(lon1, lon2) - margin_deg_lon
        maxlon = max(lon1, lon2) + margin_deg_lon
        return minlat, maxlat, minlon, maxlon

    @staticmethod
    def parse_atc_route(atc_str):
        tokens = [t.strip().strip(',') for t in atc_str.strip().split() if t.strip()]
        origin = origin_rwy = dest = dest_rwy = None
        fixes = []

        for t in tokens:
            if '/' in t:
                left, right = t.split('/', 1)
                if origin is None:
                    origin, origin_rwy = left, right
                else:
                    dest, dest_rwy = left, right
            elif t.upper() == 'DCT':
                continue
            elif re.match(r'^[A-Z]\d+$', t):
                continue
            else:
                fixes.append(t)
        return {"origin": origin, "origin_rwy": origin_rwy, "fixes": fixes, "dest": dest, "dest_rwy": dest_rwy}

    @staticmethod
    def create_route_from_atc(atc_str):
        parsed = Utils.parse_atc_route(atc_str)
        route = []
        if parsed["origin"]:
            route.append(parsed["origin"])
        route.extend(parsed["fixes"])
        if parsed["dest"]:
            route.append(parsed["dest"])
        seen = set()
        out = []
        for w in route:
            if w and w not in seen:
                out.append(w)
                seen.add(w)
        return out, parsed

    @staticmethod
    def format_dms(lat, lon, alt_m=0.0):
        def _dms(value):
            d = int(abs(value))
            m = int((abs(value) - d) * 60)
            s = (abs(value) - d - m/60.0) * 3600.0
            return d, m, s

        lat_d, lat_m, lat_s = _dms(lat)
        lon_d, lon_m, lon_s = _dms(lon)
        lat_hem = 'N' if lat >= 0 else 'S'
        lon_hem = 'E' if lon >= 0 else 'W'
        alt_str = f"+{alt_m:07.2f}"
        lat_str = f"{lat_hem}{lat_d}° {lat_m}' {lat_s:.2f}\""
        lon_str = f"{lon_hem}{lon_d}° {lon_m}' {lon_s:.2f}\""
        return f"{lat_str},{lon_str},{alt_str}"

    @staticmethod
    def get_close_waypoints(nav, lat=0, lon=0, path=None):
        """Find the closest ATC waypoint in a PLN file to (lat, lon) using the provided NavGraph-like object
        for coordinate lookups. Returns a dict {'none': ident, 'idx': index} and prints the result.
        """
        if path is None:
            raise ValueError("path is required")
        fp_el = etree.parse(path).getroot().find('FlightPlan.FlightPlan')
        if fp_el is None:
            raise ValueError("Invalid PLN file: missing FlightPlan.FlightPlan")
        atc = fp_el.findall('ATCWaypoint')
        close_waypoints = {'id': None, 'idx': 0}
        prev_distance = float('inf')
        for waypoint in atc:
            wp_id = waypoint.get('id')
            wp_lat, wp_lon = (nav.get_coords(wp_id) if hasattr(nav, 'get_coords') else (None, None))
            if wp_lat is None or wp_lon is None:
                continue
            distance = Utils.haversine(lat, lon, wp_lat, wp_lon)
            if distance < prev_distance:
                prev_distance = distance
                close_waypoints['id'] = wp_id
                close_waypoints['idx'] = atc.index(waypoint)
        #print(f"Closest waypoint to ({lat},{lon}) is {close_waypoints['id']} at index {close_waypoints['idx']} with distance {prev_distance:.1f} km")
        return close_waypoints

# ==========================
# Main components (single-file, class-based)
# ==========================

class DBInterface:
    def __init__(self, db_path):
        self.engine = create_engine(f"sqlite:///{db_path}")

    def read_sql(self, query, params=None):
        return pd.read_sql(query, self.engine, params=params or {})

class NavGraph:
    def __init__(self, db: DBInterface, dep=None, dest=None, max_distance_km=MAX_DISTANCE_KM, max_edge_km=MAX_EDGE_KM, margin_km=18000, near_airport_km=1600):
        self.db = db
        self.dep = dep
        self.dest = dest
        self.max_distance_km = max_distance_km
        self.max_edge_km = max_edge_km
        self.margin_km = margin_km
        self.near_airport_km = near_airport_km

        self.G = nx.Graph()
        self.generated_fix_coords = {}

        self._load_airports()
        self._load_waypoints()
        self._load_airways()
        self._build_graph()
        self.wp_dict = self.waypoints[['ident','laty','lonx','region']].to_dict('index')
        self.ident_to_wp_id = {v['ident']: k for k, v in self.wp_dict.items()}

    def _load_airports(self):
        q = """
        SELECT ident, laty, lonx
        FROM airport
        WHERE ident IN (:dep, :dest)
        """
        #print(f"Loading airport data for dep={self.dep}, dest={self.dest}")
        airports = self.db.read_sql(q, params={"dep": self.dep, "dest": self.dest})
        if len(airports) < 2:
            raise RuntimeError('Airports table missing or airport idents not found')

        airports_idx = airports.set_index("ident")        
        self.dep_lat, self.dep_lon = airports_idx.loc[self.dep, ['laty', 'lonx']]
        self.dst_lat, self.dst_lon = airports_idx.loc[self.dest, ['laty', 'lonx']]

        route_km = Utils.haversine(self.dep_lat, self.dep_lon, self.dst_lat, self.dst_lon)
        if route_km > 2 * self.max_distance_km:
            self.minlat, self.maxlat, self.minlon, self.maxlon = Utils.bounding_box_route(self.dep_lat, self.dep_lon, self.dst_lat, self.dst_lon, self.max_distance_km)
        else:
            self.minlat, self.maxlat, self.minlon, self.maxlon = Utils.bounding_box(self.dep_lat, self.dep_lon, self.max_distance_km)

    def _load_waypoints(self):
        q_bbox = """
        SELECT waypoint_id, ident, laty, lonx
        FROM waypoint
        WHERE laty BETWEEN :minlat AND :maxlat
        AND lonx BETWEEN :minlon AND :maxlon
        """
        bbox = self.db.read_sql(q_bbox, params={"minlat": self.minlat, "maxlat": self.maxlat, "minlon": self.minlon, "maxlon": self.maxlon})
        q_airways = """
        SELECT DISTINCT w.waypoint_id, w.ident, w.laty, w.lonx, w.region
        FROM waypoint w
        JOIN airway a ON a.from_waypoint_id = w.waypoint_id OR a.to_waypoint_id = w.waypoint_id
        """
        airways_wps = self.db.read_sql(q_airways)
        wps = pd.concat([bbox.assign(region=None), airways_wps], ignore_index=True).fillna('')
        wps = wps.drop_duplicates(subset=['waypoint_id'], keep='first').reset_index(drop=True)

        route_km = Utils.haversine(self.dep_lat, self.dep_lon, self.dst_lat, self.dst_lon)
        #if route_km > self.max_distance_km * 2:
        #    print(f"Note: route is {route_km:.0f} km, larger than bbox; candidate airway waypoints available: {len(airways_wps)}")

        wps['d_to_dep'] = wps.apply(lambda r: Utils.haversine(self.dep_lat, self.dep_lon, r['laty'], r['lonx']), axis=1)
        wps['d_to_dst'] = wps.apply(lambda r: Utils.haversine(self.dst_lat, self.dst_lon, r['laty'], r['lonx']), axis=1)
        wps['on_corridor'] = (
            (wps['d_to_dep'] + wps['d_to_dst'] - route_km) <= self.margin_km
        ) | (wps['d_to_dep'] <= self.near_airport_km) | (wps['d_to_dst'] <= self.near_airport_km)

        selected = wps[wps['on_corridor']].copy()

        #print(f"Selected {len(selected)} corridor waypoints (margin={self.margin_km} km, near airports<= {self.near_airport_km} km) out of {len(wps)} candidate waypoints")
        self.waypoints = selected.drop(columns=['d_to_dep','d_to_dst','on_corridor']).reset_index(drop=True)
        self.waypoints = self.waypoints.set_index('waypoint_id')

        #dup_idents = self.waypoints['ident'][self.waypoints['ident'].duplicated(keep=False)].unique()
        #if dup_idents.size > 0:
        #    print(f"Warning: {len(dup_idents)} duplicate waypointident(s) found: {', '.join(dup_idents[:10])}{'...' if len(dup_idents) > 10 else ''}. Keeping first occurrence of each.")

    def _load_airways(self):
        q = """
        SELECT a.airway_name, a.from_waypoint_id AS from_wp, a.to_waypoint_id AS to_wp
        FROM airway a
        """
        self.airways = self.db.read_sql(q)

    def _build_graph(self):
        valid_ids = set(self.waypoints.index)
        air = self.airways[self.airways['from_wp'].isin(valid_ids) & self.airways['to_wp'].isin(valid_ids)].copy()
        air = air.merge(self.waypoints[['ident','laty','lonx']].rename(columns={'ident':'from_ident','laty':'from_laty','lonx':'from_lonx'}), left_on='from_wp', right_index=True)
        air = air.merge(self.waypoints[['ident','laty','lonx']].rename(columns={'ident':'to_ident','laty':'to_laty','lonx':'to_lonx'}), left_on='to_wp', right_index=True)
        air['dist'] = air.apply(lambda r: Utils.haversine(r['from_laty'], r['from_lonx'], r['to_laty'], r['to_lonx']), axis=1)

        long_edges = air[air['dist'] > self.max_edge_km]
        short_edges = air[air['dist'] <= self.max_edge_km]
        #print(f"Airway records: total={len(air)}, short(<= {self.max_edge_km} km)={len(short_edges)}, long(> {self.max_edge_km} km)={len(long_edges)}")

        edges = list(zip(short_edges['from_wp'], short_edges['to_wp'], short_edges['dist']))
        self.G.add_weighted_edges_from(edges)
        #print(f"Added {len(edges)} airway edges (<= {self.max_edge_km} km)")

        self.long_edge_list = list(zip(long_edges['from_wp'], long_edges['to_wp'], long_edges['dist']))
        #print(f"Graph: nodes={self.G.number_of_nodes()}, edges={self.G.number_of_edges()}")


    def nearest_wp(self, lat, lon):
        candidates = {k: v for k, v in self.wp_dict.items() if k in self.G.nodes}
        if not candidates:
            raise RuntimeError("No waypoint candidates available in the airway graph to compute a route")
        return min(candidates.items(), key=lambda x: Utils.haversine(lat, lon, x[1]['laty'], x[1]['lonx']))[0]

    def connect_airport(self, ident, lat, lon, k=8, max_km=5000):
        
        if ident not in self.G:
            self.G.add_node(ident)
        candidates = []
        

        for n, p in self.wp_dict.items():
            if n == "AA":
                print(n, p['laty'], p['lonx'])

            if n in self.G.nodes:
                d = Utils.haversine(lat, lon, p['laty'], p['lonx'])
                candidates.append((n, d))

        if not candidates:
            print(f"Warning: no airway waypoints available to connect airport {ident}")
            return
        candidates.sort(key=lambda x: x[1])
        selected = candidates[:k]
        did_connect = 0

        for node, dist in selected:
            if dist <= max_km or did_connect == 0:
                self.G.add_edge(ident, node, weight=dist)
                did_connect += 1
        

        #print(f"Connected {ident} to {did_connect} airway node(s); nearest distance {selected[0][1]:.1f} km")

    def connect_fix(self, ident, lat, lon, k=8, max_km=5000, forward_bearing=None, cone_deg=120):
        if ident not in self.G:
            self.G.add_node(ident)
        candidates = []
        
        for n, p in self.wp_dict.items():
            if n in self.G.nodes:
                d = Utils.haversine(lat, lon, p['laty'], p['lonx'])
                b = Utils.bearing_deg(lat, lon, p['laty'], p['lonx'])
                candidates.append((n, d, b))
        if not candidates:
            print(f"Warning: no airway waypoints available to connect fix {ident}")
            return
        if forward_bearing is not None:
            half = cone_deg / 2.0
            def within_cone(b):
                diff = abs(((b - forward_bearing + 180) % 360) - 180)
                return diff <= half
            cone_candidates = [c for c in candidates if within_cone(c[2])]
            if cone_candidates:
                candidates = cone_candidates
        candidates.sort(key=lambda x: x[1])
        selected = candidates[:k]
        did_connect = 0
        for node, dist, _ in selected:
            if dist <= max_km or did_connect == 0:
                self.G.add_edge(ident, node, weight=dist)
                did_connect += 1
        #print(f"Connected fix {ident} to {did_connect} airway node(s); nearest distance {selected[0][1]:.1f} km")


    def ensure_node_connected(self, node, is_start=True):
        if node in self.G:
            return node
        lat, lon = (self.dep_lat, self.dep_lon) if node == self.dep else ((self.dst_lat, self.dst_lon) if node == self.dest else self.get_coords(node))
        if lat is not None:
            fb = Utils.bearing_deg(lat, lon, self.dst_lat, self.dst_lon) if is_start else Utils.bearing_deg(lat, lon, self.dep_lat, self.dep_lon)
            self.connect_fix(node, lat, lon, forward_bearing=fb)
            if node in self.G:
                return node
        substitute = self.nearest_wp(self.dep_lat if is_start else self.dst_lat, self.dep_lon if is_start else self.dst_lon)
        #print(f"Warning: node {node} not connected or missing coordinates; using nearest airway node {substitute} instead")
        return substitute

    def get_coords(self, ident):
        if isinstance(ident, int) or (isinstance(ident, str) and ident.isdigit()):
            wp_id = int(ident)
            if wp_id in self.wp_dict:
                p = self.wp_dict[wp_id]
                return p['laty'], p['lonx']

        if ident in self.generated_fix_coords:
            return self.generated_fix_coords[ident]
        if ident == self.dep:
            return self.dep_lat, self.dep_lon
        if ident == self.dest:
            return self.dst_lat, self.dst_lon

        lookups = [
            ("waypoint", "laty", "lonx"),
            ("vor", "laty", "lonx"),
            ("ndb", "laty", "lonx"),
            ("ils", "laty", "lonx"),
            ("airport", "laty", "lonx"),
        ]
        for table, latcol, loncol in lookups:
            try:
                q = f"SELECT {latcol} as laty, {loncol} as lonx FROM {table} WHERE ident = :ident LIMIT 1"
                r = self.db.read_sql(q, params={"ident": ident})
                if not r.empty and pd.notna(r.iloc[0]['laty']):
                    return float(r.iloc[0]['laty']), float(r.iloc[0]['lonx'])
            except Exception:
                continue
        try:
            r = self.db.read_sql("SELECT laty, lonx FROM waypoint WHERE LOWER(ident) = LOWER(:ident) LIMIT 1", params={"ident": ident})
            if not r.empty:
                return float(r.iloc[0]['laty']), float(r.iloc[0]['lonx'])
        except Exception:
            pass
        return None, None

class SidStarExtractor:
    def __init__(self, navgraph: NavGraph, preferred_sid_name=None, preferred_star_name=None, preferred_sid_runway=None):
        self.nav = navgraph
        self.db = navgraph.db
        self.wp_dict = navgraph.wp_dict
        self.G = navgraph.G
        self.generated_fix_coords = navgraph.generated_fix_coords
        self.preferred_sid_name = preferred_sid_name
        self.preferred_star_name = preferred_star_name
        self.preferred_sid_runway = preferred_sid_runway

    def extract_sid_fixes(self, airport_ident):
        fixes = []
        try:
            #print(f"Extracting SID fixes for airport {airport_ident} with preferred SID name '{self.preferred_sid_name}' and preferred runway '{self.preferred_sid_runway}'")
            apps = self.db.read_sql("SELECT approach_id, arinc_name, fix_ident FROM approach WHERE airport_ident = :dep AND fix_ident = :sid_name", params={"dep": airport_ident,"sid_name": self.preferred_sid_name })
            if not apps.empty:
                candidates = []
                airway_nodes = {n: p for n, p in self.wp_dict.items() if n in self.G.nodes}
                def nearest_node_dist(lat, lon):
                    return min(Utils.haversine(lat, lon, p['laty'], p['lonx']) for p in airway_nodes.values()) if airway_nodes else float('inf')
                for _, a in apps.iterrows():
                    aid = int(a['approach_id'])
                    aname = a.get('arinc_name')
                    seq = []
                    dists = []
                    if pd.notna(a.get('fix_ident')):
                        seq.append(a['fix_ident'])
                        if a['fix_ident'] in self.nav.ident_to_wp_id:
                            p = self.wp_dict[self.nav.ident_to_wp_id[a['fix_ident']]]
                            dists.append(nearest_node_dist(p['laty'], p['lonx']))
                    
                    legs = self.db.read_sql("SELECT approach_leg_id, fix_ident, fix_laty, fix_lonx FROM approach_leg WHERE approach_id = :aid ORDER BY approach_leg_id", params={"aid": aid})
                    for _, r in legs.iterrows():
                        if pd.notna(r.get('fix_ident')):
                            seq.append(r['fix_ident'])
                            if r['fix_ident'] in self.nav.ident_to_wp_id:
                                p = self.wp_dict[self.nav.ident_to_wp_id[r['fix_ident']]]
                                dists.append(nearest_node_dist(p['laty'], p['lonx']))
                        elif pd.notna(r.get('fix_laty')) and pd.notna(r.get('fix_lonx')):
                            name = f"{airport_ident}_APP_{aid}_{int(r['approach_leg_id'])}"
                            self.generated_fix_coords[name] = (float(r['fix_laty']), float(r['fix_lonx']))
                            seq.append(name)
                            dists.append(nearest_node_dist(float(r['fix_laty']), float(r['fix_lonx'])))
                    if seq:
                        last_fix = seq[-1]
                        last_latlon = None
                        if last_fix in self.nav.ident_to_wp_id:
                            last_latlon = (self.wp_dict[self.nav.ident_to_wp_id[last_fix]]['laty'], self.wp_dict[self.nav.ident_to_wp_id[last_fix]]['lonx'])
                        elif last_fix in self.generated_fix_coords:
                            last_latlon = self.generated_fix_coords[last_fix]
                        last_dist = nearest_node_dist(last_latlon[0], last_latlon[1]) if last_latlon else float('inf')
                        avg_dist = sum(dists)/len(dists) if dists else float('inf')
                        candidates.append((aid, aname, seq, last_dist, avg_dist))
                if candidates:
                    if self.preferred_sid_name:
                        # match preferred SID case-insensitively against arinc_name and any fix idents in the sequence
                        pname = self.preferred_sid_name.strip().upper()
                        matched = []
                        for c in candidates:
                            aid, aname, seq, lastd, avgd = c
                            # exact or substring match against arinc_name
                            if aname and isinstance(aname, str):
                                aname_up = aname.strip().upper()
                                if aname_up == pname or pname in aname_up:
                                    matched.append(c)
                                    continue
                            # check if preferred name appears in the fix sequence
                            for s in seq:
                                if isinstance(s, str) and s.strip().upper() == pname:
                                    matched.append(c)
                                    break
                        if matched:
                            # prefer candidate with smaller last-fix or average distance
                            matched.sort(key=lambda x: (x[3], x[4]))
                            chosen = matched[0]
                            #print(f"Selected approach {chosen[0]} ({chosen[1]}) as SID (preferred name matched)")
                            return chosen[2]
                    # fallback: choose candidate with smallest last-fix distance then avg distance
                    candidates.sort(key=lambda x: (x[3], x[4]))
                    chosen = candidates[0]
                    #print(f"Selected approach {chosen[0]} ({chosen[1]}) as SID (last-fix dist {chosen[3]:.1f} km, avg {chosen[4]:.1f} km)")
                    return chosen[2]
        except Exception:
            pass

        try:
            trans = self.db.read_sql("SELECT transition_id, type, fix_ident FROM transition WHERE fix_airport_ident = :dep", params={"dep": airport_ident})
            if not trans.empty:
                candidates = []
                airway_nodes = {n: p for n, p in self.wp_dict.items() if n in self.G.nodes}
                def nearest_node_dist(lat, lon):
                    return min(Utils.haversine(lat, lon, p['laty'], p['lonx']) for p in airway_nodes.values()) if airway_nodes else float('inf')
                for _, t in trans.iterrows():
                    tid = int(t['transition_id'])
                    legs = self.db.read_sql("SELECT transition_leg_id, fix_ident, fix_laty, fix_lonx FROM transition_leg WHERE transition_id = :tid ORDER BY transition_leg_id", params={"tid": tid})
                    seq = []
                    dists = []

                    for _, r in legs.iterrows():
                        if pd.notna(r.get('fix_ident')):
                            seq.append(r['fix_ident'])
                            if r['fix_ident'] in self.nav.ident_to_wp_id:
                                p = self.wp_dict[self.nav.ident_to_wp_id[r['fix_ident']]]
                                dists.append(nearest_node_dist(p['laty'], p['lonx']))
                        elif pd.notna(r.get('fix_laty')) and pd.notna(r.get('fix_lonx')):
                            name = f"{airport_ident}_TRANS_{tid}_{int(r['transition_leg_id'])}"
                            self.generated_fix_coords[name] = (float(r['fix_laty']), float(r['fix_lonx']))
                            seq.append(name)
                            dists.append(nearest_node_dist(float(r['fix_laty']), float(r['fix_lonx'])))
                    if seq:
                        avg_dist = sum(dists)/len(dists) if dists else float('inf')
                        candidates.append((tid, seq, avg_dist))
                if candidates:
                    candidates.sort(key=lambda x: x[2])
                    chosen = candidates[0]
                    #print(f"Selected transition {chosen[0]} as SID (avg dist to airway nodes {chosen[2]:.1f} km)")
                    return chosen[1]
        except Exception:
            pass

        try:
            starts = self.db.read_sql("SELECT start_id, runway_name, lonx, laty FROM start WHERE airport_id IN (SELECT airport_id FROM airport WHERE ident = :dep)", params={"dep": airport_ident})
            for _, r in starts.iterrows():
                if pd.isna(r.get('laty')) or pd.isna(r.get('lonx')):
                    continue
                # If a preferred runway is set, only use matching starts
                if self.preferred_sid_runway and pd.notna(r.get('runway_name')) and r['runway_name'] != self.preferred_sid_runway:
                    continue
                name = f"{airport_ident}_START_{int(r['start_id'])}"
                self.generated_fix_coords[name] = (float(r['laty']), float(r['lonx']))
                fixes.append(name)
            if fixes:
                #if self.preferred_sid_runway:
                #    print(f"Fallback: using runway start positions for preferred runway {self.preferred_sid_runway} as SID-like fixes ({len(fixes)} entries)")
                #else:
                #    print(f"Fallback: using runway start positions as SID-like fixes ({len(fixes)} entries)")
                return fixes
        except Exception:
            pass

        try:
            trans = self.db.read_sql("SELECT DISTINCT fix_ident FROM transition WHERE fix_airport_ident = :dep", params={"dep": airport_ident})
            fixes += trans['fix_ident'].dropna().tolist()
        except Exception:
            pass

        return fixes

    def extract_star_fixes(self, airport_ident, preferred_name=None):
        fixes = []
        try:
            apps = self.db.read_sql("SELECT approach_id, arinc_name, fix_ident FROM approach WHERE airport_ident = :dest ", params={"dest": airport_ident})
            if not apps.empty:
                chosen_aid = None
                if preferred_name:
                    matched = apps[apps['fix_ident'] == preferred_name]
                    if not matched.empty:
                        chosen_aid = int(matched.iloc[0]['approach_id'])
                if chosen_aid is None:
                    counts = {}
                    for _, a in apps.iterrows():
                        aid = int(a['approach_id'])
                        legs = self.db.read_sql("SELECT COUNT(*) as cnt FROM approach_leg WHERE approach_id = :aid", params={"aid": aid})
                        counts[aid] = int(legs.iloc[0]['cnt'])
                    chosen_aid = max(counts, key=counts.get)
                app_row = apps[apps['approach_id'] == chosen_aid].iloc[0]
                if pd.notna(app_row.get('fix_ident')):
                    fixes.append(app_row['fix_ident'])
                legs = self.db.read_sql("SELECT fix_ident, fix_lonx, fix_laty FROM approach_leg WHERE approach_id = :aid ORDER BY approach_leg_id", params={"aid": chosen_aid})
                for _, r in legs.iterrows():
                    if pd.notna(r.get('fix_ident')):
                        fixes.append(r['fix_ident'])
                    elif pd.notna(r.get('fix_laty')) and pd.notna(r.get('fix_lonx')):
                        name = f"{airport_ident}_APP_LEG_{len(fixes)+1}"
                        self.generated_fix_coords[name] = (float(r['fix_laty']), float(r['fix_lonx']))
                        fixes.append(name)
                if fixes:
                    #print(f"Selected approach id {chosen_aid} for STAR with {len(fixes)} fixes")
                    return fixes
        except Exception:
            pass

        try:
            trans = self.db.read_sql("SELECT transition_id, fix_ident FROM transition WHERE fix_airport_ident = :dest", params={"dest": airport_ident})
            for _, t in trans.iterrows():
                if pd.notna(t.get('fix_ident')):
                    fixes.append(t['fix_ident'])
                tl = self.db.read_sql("SELECT fix_ident, fix_laty, fix_lonx FROM transition_leg WHERE transition_id = :tid ORDER BY transition_leg_id", params={"tid": int(t['transition_id'])})
                for _, r in tl.iterrows():
                    if pd.notna(r.get('fix_ident')):
                        fixes.append(r['fix_ident'])
                    elif pd.notna(r.get('fix_laty')) and pd.notna(r.get('fix_lonx')):
                        name = f"{airport_ident}_TRANS_LEG_{len(fixes)+1}"
                        self.generated_fix_coords[name] = (float(r['fix_laty']), float(r['fix_lonx']))
                        fixes.append(name)
            return fixes
        except Exception:
            pass

        return fixes

class RouteBuilder:
    def __init__(self, nav: NavGraph, sidstar: SidStarExtractor, include_sid=False, include_star=False):
        self.nav = nav
        self.sidstar = sidstar
        self.include_sid = include_sid
        self.include_star = include_star
        self.sid_fixes = []
        self.star_fixes = []
        self.path = None

    def load_sid_star(self):
        if self.include_sid:
            self.sid_fixes = self.sidstar.extract_sid_fixes(self.nav.dep)
            #if self.sid_fixes:
            #    print(f"Using SID fixes: {self.sid_fixes[:10]}{'...' if len(self.sid_fixes)>10 else ''}")
            #else:
            #    print("No SID fixes found for airport; continuing without SID.")
        else:
            self.sid_fixes = []
           # print("Skipping SID extraction (not requested by user).")

        if self.include_star:
            self.star_fixes = self.sidstar.extract_star_fixes(self.nav.dest, preferred_name=self.sidstar.preferred_star_name)
            #if self.star_fixes:
            #    print(f"Using STAR fixes: {self.star_fixes[:10]}{'...' if len(self.star_fixes)>10 else ''}")
            #else:
            # print("No STAR fixes found for airport; continuing without STAR.")
        else:
            self.star_fixes = []
            #print("Skipping STAR extraction (not requested by user).")

    def build_route(self):
        # connect airports
        self.nav.connect_airport(self.nav.dep, self.nav.dep_lat, self.nav.dep_lon)
        self.nav.connect_airport(self.nav.dest, self.nav.dst_lat, self.nav.dst_lon)

        # connect SID fixes
        if self.sid_fixes:
            for i, f in enumerate(self.sid_fixes):
                lat, lon = self.nav.get_coords(f)
                if lat is None:
                #    print(f"Warning: no coordinates for fix {f}")
                    continue
                if i+1 < len(self.sid_fixes):
                    nlat, nlon = self.nav.get_coords(self.sid_fixes[i+1])
                    forward = Utils.bearing_deg(lat, lon, nlat, nlon) if nlat is not None else None
                else:
                    forward = Utils.bearing_deg(lat, lon, self.nav.dst_lat, self.nav.dst_lon)
                self.nav.connect_fix(f, lat, lon, forward_bearing=forward)

        # connect first STAR fix
        if self.star_fixes:
            first_star = self.star_fixes[0]
            lat, lon = self.nav.get_coords(first_star)
            if lat is not None:
                forward = Utils.bearing_deg(lat, lon, self.nav.dst_lat, self.nav.dst_lon)
                self.nav.connect_fix(first_star, lat, lon, forward_bearing=forward)

        start_node = self.sid_fixes[-1] if self.sid_fixes else self.nav.dep
        end_node = self.star_fixes[0] if self.star_fixes else self.nav.dest


        start_node = self.nav.ensure_node_connected(start_node, is_start=True)
        end_node = self.nav.ensure_node_connected(end_node, is_start=False)


        #print(f"Computing airway subpath from {start_node} to {end_node} using graph nodes: {len(self.nav.G.nodes)} nodes, {len(self.nav.G.edges)} edges")
        try:
            self.path = nx.shortest_path(self.nav.G, start_node, end_node, weight='weight')
        except nx.NodeNotFound as e:
            print(f"NodeNotFound: {e}; graph has {self.nav.G.number_of_nodes()} nodes. Sample nodes: {list(self.nav.G.nodes)[:10]}")
            raise
        except nx.NetworkXNoPath:
            print("No path found between SID/STAR with filtered edges; adding long airway edges (fallback) and retrying")
            self.nav.G.add_weighted_edges_from(self.nav.long_edge_list)
            try:
                self.path = nx.shortest_path(self.nav.G, start_node, end_node, weight='weight')
                #print(self.path)
            except nx.NetworkXNoPath:
                print("Still no path found; adding direct great circle route")
                start_lat, start_lon = self.nav.get_coords(start_node)
                end_lat, end_lon = self.nav.get_coords(end_node)
                if start_lat is not None and end_lat is not None:
                    dist = Utils.haversine(start_lat, start_lon, end_lat, end_lon)
                    self.nav.G.add_edge(start_node, end_node, weight=dist)
                    self.path = [start_node, end_node]
                else:
                    raise RuntimeError("No coordinates for start or end node, cannot add direct route")

        if self.path and self.path[0] == start_node:
            self.path = self.path[1:]
        if self.path and self.path[-1] == end_node:
            self.path = self.path[:-1]
              

        # Connect 1st STAR fix to closest waypoint
        if self.star_fixes:
            first_star = self.star_fixes[0]
            lat_lon = self.nav.get_coords(first_star)
            if lat_lon and lat_lon[0] is not None:
                lat, lon = lat_lon
                # Prefer connecting to a node in the computed path for consistency; fall back to any graph node
                target_nodes = list(self.path) if self.path else list(self.nav.G.nodes)
                best = None
                best_dist = float('inf')
                for n in target_nodes:
                    nlat, nlon = self.nav.get_coords(n)
                    if nlat is None:
                        continue
                    d = Utils.haversine(lat, lon, nlat, nlon)
                    if d < best_dist:
                        best_dist = d
                        best = n
                if best:
                    if first_star not in self.nav.G:
                        self.nav.G.add_node(first_star)
                    # (re)connect with weight equal to great-circle distance
                    self.nav.G.add_edge(first_star, best, weight=best_dist)
                   # print(f"Connected first STAR fix {first_star} to nearest path node {best} (dist {best_dist:.1f} km)")
                #else:
                   # print(f"Warning: no suitable node found to connect first STAR fix {first_star}")
            #else:
                #print(f"Warning: no coordinates for first STAR fix {first_star}")

     
    def merge_full_route(self):
        route = [self.nav.dep] + self.sid_fixes + (self.path or []) + self.star_fixes + [self.nav.dest]
        route = [r for r in route if r]
        route = [x for x in route if x != self.nav.dep]
        route.insert(0, self.nav.dep)
        route = [x for x in route if x != self.nav.dest]
        seen = set()
        dedup = []
        for wp in route:
            if wp not in seen:
                dedup.append(wp)
                seen.add(wp)
        removed = len(route) - len(dedup)
        #if removed:
        #    print(f"Removed {removed} duplicated waypoint(s) from route")
        dedup.append(self.nav.dest)
        final_route = []
        for wp in dedup:
            if not final_route or final_route[-1] != wp:
                final_route.append(wp)
        #if self.nav.dest in final_route[:-1]:
        #    print(f"Warning: destination {self.nav.dest} found in middle of route; it was moved to the end")
        
        
        return final_route
    

class PLNWriter:
    def __init__(self, nav: NavGraph, dep_name=None, dest_name=None, dep_elev=0.0, dest_elev=0.0, cruise_fl=CRUISE_FL):
        self.nav = nav
        self.dep_name = dep_name or nav.dep
        self.dest_name = dest_name or nav.dest
        self.dep_elev = dep_elev
        self.dest_elev = dest_elev
        self.cruise_fl = cruise_fl
        self.route_type = 'HighAlt' if cruise_fl >= 180 else 'LowAlt'
        self.cruise_alt_ft = int(cruise_fl)

    def _build_root(self, dep, dest):
        root = etree.Element('SimBase.Document', Type='AceXML', version='1,0')
        etree.SubElement(root, 'Descr').text = 'AceXML Document'
        fp = etree.SubElement(root, 'FlightPlan.FlightPlan')
        etree.SubElement(fp, 'Title').text = f'{dep} to {dest}'
        etree.SubElement(fp, 'FPType').text = 'IFR'
        etree.SubElement(fp, 'RouteType').text = self.route_type
        etree.SubElement(fp, 'CruisingAlt').text = str(self.cruise_alt_ft)
        etree.SubElement(fp, 'DepartureID').text = dep
        try:
            etree.SubElement(fp, 'DepartureLLA').text = Utils.format_dms(self.nav.dep_lat, self.nav.dep_lon, self.dep_elev)
        except Exception:
            etree.SubElement(fp, 'DepartureLLA').text = ''
        etree.SubElement(fp, 'DestinationID').text = dest
        try:
            etree.SubElement(fp, 'DestinationLLA').text = Utils.format_dms(self.nav.dst_lat, self.nav.dst_lon, self.dest_elev)
        except Exception:
            etree.SubElement(fp, 'DestinationLLA').text = ''
        etree.SubElement(fp, 'DepartureName').text = self.dep_name
        etree.SubElement(fp, 'DestinationName').text = self.dest_name
        appv = etree.SubElement(fp, 'AppVersion')
        etree.SubElement(appv, 'AppVersionMajor').text = '1'
        etree.SubElement(appv, 'AppVersionBuild').text = '0'
        return root, fp


    def validate_pln(self, path):
        try:
            doc = etree.parse(path)
        except Exception as e:
            print(f'PLN parse error: {e}')
            return False
        root_el = doc.getroot()
        if root_el.tag != 'SimBase.Document' or root_el.get('Type') != 'AceXML':
            print('Invalid root element or Type attribute')
            return False
        fp_el = root_el.find('FlightPlan.FlightPlan')
        if fp_el is None:
            print('Missing FlightPlan.FlightPlan element')
            return False
        required = ['Title', 'FPType', 'RouteType', 'CruisingAlt', 'DepartureID', 'DepartureLLA', 'DestinationID', 'DestinationLLA']
        missing = [r for r in required if fp_el.find(r) is None]
        if missing:
            print('Missing required elements:', missing)
            return False
        atc = fp_el.findall('ATCWaypoint')
        if not atc:
            print('No ATCWaypoint elements found')
            return False
        return True

    def write_lnm_pln(self, file_path, route_list, dep=None, dest=None):
        """Write legacy little-navmap-style PLN (keeps previous behavior)."""
        root = etree.Element('SimBase.Document', Type='AceXML', version='1,0')
        etree.SubElement(root, 'Descr').text = 'AceXML Document'
        fp = etree.SubElement(root, 'FlightPlan.FlightPlan')
        etree.SubElement(fp, 'Title').text = f"{dep} to {dest}"
        etree.SubElement(fp, 'FPType').text = 'IFR'
        etree.SubElement(fp, 'RouteType').text = self.route_type
        etree.SubElement(fp, 'CruisingAlt').text = str(self.cruise_alt_ft)
        etree.SubElement(fp, 'DepartureID').text = dep
        etree.SubElement(fp, 'DepartureLLA').text = Utils.format_dms(self.nav.dep_lat, self.nav.dep_lon, 0.0)
        etree.SubElement(fp, 'DestinationID').text = dest
        etree.SubElement(fp, 'DestinationLLA').text = Utils.format_dms(self.nav.dst_lat, self.nav.dst_lon, 0.0)
        etree.SubElement(fp, 'Descr').text = f"{dep}, {dest} created by littlenav.py"
        etree.SubElement(fp, 'DepartureName').text = self.dep_name
        etree.SubElement(fp, 'DestinationName').text = self.dest_name
        appv = etree.SubElement(fp, 'AppVersion')
        etree.SubElement(appv, 'AppVersionMajor').text = '1'
        etree.SubElement(appv, 'AppVersionBuild').text = '0'

        added = set()
        for wp in route_list:
            if wp in added:
                continue

            if isinstance(wp, int) or (isinstance(wp, str) and wp.isdigit()):
                ident = self.nav.wp_dict[int(wp)]['ident']
                lat_lon = self.nav.get_coords(wp)
            else:
                ident = wp
                lat_lon = self.nav.get_coords(wp)

            if lat_lon and lat_lon[0] is not None:
              added.add(str(wp))
              atc = etree.SubElement(fp, 'ATCWaypoint', id=ident)
              wp_type = 'Airport' if ident in (dep, dest) else 'Intersection'
              etree.SubElement(atc, 'ATCWaypointType').text = wp_type
              lat, lon = lat_lon
              wpos = Utils.format_dms(lat, lon, self.cruise_alt_ft) if wp_type == 'Intersection' else Utils.format_dms(lat, lon, 0.0)
              etree.SubElement(atc, 'WorldPosition').text = wpos
              ica = etree.SubElement(atc, 'ICAO')
              etree.SubElement(ica, 'ICAOIdent').text = ident

        out_tree = etree.ElementTree(root)
        out_tree.write(file_path, pretty_print=True, xml_declaration=True, encoding='UTF-8')
        #print('PLN flight plan generated:', file_path)

    def get_close_waypoints(self, lat=0, lon=0, path=None):
        """Deprecated wrapper delegating to Utils.get_close_waypoints(nav, ...)."""
        return Utils.get_close_waypoints(self.nav, lat=lat, lon=lon, path=path)

# ==========================
# CLI / Controller
# ==========================

def parse_bool_env(name):
    v = os.getenv(name)
    if v is None:
        return None
    v = v.strip().lower()
    if v in ("1", "true", "yes", "y", "on"):
        return True
    if v in ("0", "false", "no", "n", "off"):
        return False
    return None


def generate_route(db_path, dep=None, dest=None, include_sid=None, include_star=None):
    """Build nav objects and return (db, nav, writer, route).

    include_sid/include_star should be resolved by the caller (env/interactive).
    """
    db = DBInterface(db_path)
    nav = NavGraph(db, dep=dep, dest=dest)

    if include_sid == True or include_star == True:
        sidstar = SidStarExtractor(nav, preferred_sid_name=PREFERRED_SID_NAME, preferred_star_name=PREFERRED_STAR_NAME, preferred_sid_runway=PREFERRED_SID_RUNWAY)
    else:
        sidstar = None
    
    rb = RouteBuilder(nav, sidstar, include_sid=include_sid, include_star=include_star)
    
    if sidstar != None:
        rb.load_sid_star()
    
    rb.build_route()
    route = rb.merge_full_route()
    writer = PLNWriter(nav)
    return db, nav, writer, route


def Create_plan_with_close_waypoint(db_path,dep=None, dest=None, output_file=OUTPUT_FILE, include_sid=None, include_star=None,lat=None, lon=None,altitude=33000, Dep_RW = None, Des_RW = None, Dep_SID = None, Des_STAR = None):

    
    CRUISE_FL = altitude
    
    if Dep_RW != None:
        PREFERRED_DEP_RW = Dep_RW

    if Des_RW != None:
        PREFERRED_DES_RW = Des_RW
    
    if Dep_SID != None:
        PREFERRED_SID = Dep_SID
    
    if Dep_SID != None:
        PREFERRED_STAR = Des_STAR

    # build route using helper
    db, nav, writer, route = generate_route(db_path, dep=dep, dest=dest, include_sid=include_sid, include_star=include_star)

    writer = PLNWriter(nav)
    writer.write_lnm_pln(output_file, route, dep=dep, dest=dest)
    ok = writer.validate_pln(output_file)
    #print('Validation:', 'OK ✅' if ok else 'FAILED ⚠️')
    #print('Route:')

    # Get display names for the route
    route_names = []
    for wp in route:
        if isinstance(wp, int) or (isinstance(wp, str) and wp.isdigit()):
            ident = nav.wp_dict[int(wp)]['ident']
            route_names.append(ident)
        else:
            route_names.append(wp)

    #print(' '.join(route_names))

    final_route = {'Route':[] ,'id': None, 'idx': 0}
    if lat is not None and lon is not None:
      close_waypoints = Utils.get_close_waypoints(nav, lat=lat, lon=lon, path=output_file)
      final_route['id'] = close_waypoints['id']
      final_route['idx'] = close_waypoints['idx']
    final_route['Route'] = route_names
    #print(final_route)  
    return final_route

    # optional ATC route demo
    #ATC_ROUTE = "KSFO/01L DCT BEBOP R464 BITTA INOYI1 PHNL/04L"
    #if ATC_ROUTE:
    #    r, info = Utils.create_route_from_atc(ATC_ROUTE)
    #    print(r)
    #    print(info)
    #
    #return route


DB_PATH = "Database\little_navmap.sqlite"
#Test call
#close_waypoints = Create_plan_with_close_waypoint(db_path=DB_PATH,dep="NZAA", dest="NTAA", output_file="Cruise_flt.pln", include_sid=0, include_star=0,lat=50.78694534301758, lon=12.096664428710938 )
#close_waypoints = Create_plan_with_close_waypoint(db_path=DB_PATH,dep="NZAA", dest="SCIP", output_file="Cruise_flt.pln", include_sid=0, include_star=0,lat=50.78694534301758, lon=12.096664428710938 )
#print(close_waypoints)
