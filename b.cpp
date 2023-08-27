#include <cstdio>
#include <map>
#include <vector>
#include <string>
#include <queue>
#include <algorithm>
#include <cmath>

#define MAX_NODE 58005
#define ARG 1.1
#define GRIDWID 0.0016
#define STOPLEN 0.0005
#define GRIDNUM 2102

using namespace std;
template <class T>
inline bool scan_d(T &ret)
{
    char c;
    int sgn;
    if (c = getchar(), c == EOF)
        return 0; // EOF
    while (c != '-' && (c < '0' || c > '9'))
        c = getchar();
    sgn = (c == '-') ? -1 : 1;
    ret = (c == '-') ? 0 : (c - '0');
    while (c = getchar(), c >= '0' && c <= '9')
        ret = ret * 10 + (c - '0');
    ret *= sgn;
    return 1;
}

struct Posi
{
    double x, y;
};
struct Posi_t
{
    int time;
    double x, y;
};
struct Road
{
    int id, posinum;
    double roadlen;
    int from, to, lev;
    vector<Posi> vp;
};
struct Track
{
    vector<Posi_t> track;
};

int N, M;
vector<int> head[MAX_NODE];
vector<Road> edges;
vector<Track> tracks;
vector<map<int,double>> shortestlens;
// vector<map<int, int>> paths;
// vector<Posi_t> track;
// map<int, int> friend_road;
map<int, double> curprob;
map<int, int> preedge;
// 当前概率与对应的边
double hight_min, wid_min;
double height = GRIDWID, width = GRIDWID;
vector<int> grids[GRIDNUM][GRIDNUM];
// 0.01° -> 1.1km


inline double cqlt_pp_eu(const Posi &p1, const Posi &p2)
{
    return sqrt(ARG * (p1.x - p2.x) * ARG * (p1.x - p2.x) + ARG * (p1.y - p2.y) * ARG * (p1.y - p2.y));
}

void read()
{
    // 边
    // Road before;
    // before.from = before.to = 0; 
    wid_min = hight_min = 200.0;

    Posi p;
    double x_c, y_c;
    string way_string;
    scan_d(N);
    for (int ii = 0; ii < N; ii++)
    {
        Road newroad;
        scan_d(newroad.id);
        scan_d(newroad.from);
        scan_d(newroad.to);
        // cin >> way_string;
        scanf("%s", &way_string[0]);
        scan_d(newroad.lev);
        scan_d(newroad.posinum);
        newroad.roadlen = 0.0;
        for (int jj = 0; jj < newroad.posinum; jj++)
        {
            scanf("%lf%lf", &x_c, &y_c);
            p.x = x_c, p.y = y_c;
            newroad.vp.emplace_back(p);
        }
        for (int i = 0; i < newroad.vp.size() - 1; i++)
        {
            newroad.roadlen += cqlt_pp_eu(newroad.vp[i], newroad.vp[i + 1]);
            hight_min = min(hight_min, newroad.vp[i].y);
            wid_min = min(wid_min, newroad.vp[i].x);
       
        }
        edges.emplace_back(newroad);
        head[newroad.from].emplace_back(newroad.id);
        // if (newroad.to == before.from && newroad.from == before.to)
        // {
        //     friend_road[newroad.id] = before.id;
        //     friend_road[before.id] = newroad.id;
        // }
        // before.from = newroad.from, before.to = newroad.to, before.id = newroad.id;
    }

    // 轨迹
    int t;
    double x_m, y_m;
    scan_d(M);
    for (int ii = 0; ii < M; ii++)
    {
        Track tra;
        Posi_t poi;

        while (1)
        {
            scan_d(t);
            if (t == ii)
                break;
            // poi.time=ti;
            scanf("%lf%lf", &poi.x, &poi.y);
            poi.time = t;
            tra.track.emplace_back(poi);
        }
        tracks.emplace_back(tra);
    }
}

void initial()
{
    int size_edge=edges.size();
    int size_vp;
    for (int i = 0; i < size_edge; i++)
    {
        size_vp=edges[i].vp.size();
        for (int j = 0; j < size_vp; j++)
        {
            int row = int(floor(ARG * (edges[i].vp[j].y - hight_min) / height));
            int col = int(floor(ARG * (edges[i].vp[j].x - wid_min) / width));
            grids[row][col].emplace_back(i);
            grids[row][col + 1].emplace_back(i);
            grids[row][col + 2].emplace_back(i);
            grids[row + 1][col].emplace_back(i);
            grids[row + 1][col + 1].emplace_back(i);
            grids[row + 1][col + 2].emplace_back(i);
            grids[row + 2][col].emplace_back(i);
            grids[row + 2][col + 1].emplace_back(i);
            grids[row + 2][col + 2].emplace_back(i);
        }
    }

    for (int i = 0; i < GRIDNUM; i++)
    {
        for (int j = 0; j < GRIDNUM; j++)
        {
            sort(grids[i][j].begin(), grids[i][j].end());
            grids[i][j].erase(unique(grids[i][j].begin(), grids[i][j].end()), grids[i][j].end());
        }
    }
}

pair<int, int> belong_to_grid(const Posi &posi)
{
    int row = int(floor(ARG * (posi.y - hight_min) / height));
    int col = int(floor(ARG * (posi.x - wid_min) / width));
    return make_pair(row + 1, col + 1);
}


bool no_dunjiao(const Posi &posi, const Posi &posi_start, const Posi &posi_end)
{
    double x1 = posi.x - posi_end.x;
    double x2 = posi_start.x - posi_end.x;
    double y1 = posi.y - posi_end.y;
    double y2 = posi_start.y - posi_end.y;
    double cos1 = x1 * x2 + y1 * y2;

    x1 = posi_end.x - posi_start.x;
    x2 = posi.x - posi_start.x;
    y1 = posi_end.y - posi_start.y;
    y2 = posi.y - posi_start.y;
    double cos2 = x1 * x2 + y1 * y2;
    if (cos1 > 0 && cos2 > 0)
        return true;
    return false;
}

double cqlt_s(const Posi &pt, const Posi &p1, const Posi &p2)
{
    double x1 = (p1.x - pt.x) * ARG;
    double x2 = (p2.x - pt.x) * ARG;
    double y1 = (p1.y - pt.y) * ARG;
    double y2 = (p2.y - pt.y) * ARG;
    return abs((x1 * y2 - x2 * y1) / 2);
}

double cqlt_posi_to_edge(const Posi &posi, const Road &edge)
{
    double ret = 1e9 + 7.0;
    double temp = 0;
    for (int i = 1; i < edge.vp.size(); i++)
    {
        temp = min(cqlt_pp_eu(posi, edge.vp[i - 1]), cqlt_pp_eu(posi, edge.vp[i]));

        if (no_dunjiao(posi, edge.vp[i - 1], edge.vp[i]))
        {
            double S = cqlt_s(posi, edge.vp[i - 1], edge.vp[i]);
            double alt = 2.0 * S / cqlt_pp_eu(edge.vp[i - 1], edge.vp[i]);
            temp = min(temp, abs(alt));
        }
        if (ret > temp)
            ret = temp;
    }
    return ret;
}

double cqlt_start_to_posi(const Posi &cur_p, const Road &edge)
{
    double dist = 0, pre = 0;
    int sz = edge.vp.size();
    int i;
    for (i = 0; i < sz - 1 && !no_dunjiao(cur_p, edge.vp[i], edge.vp[i + 1]); i++)
        dist += cqlt_pp_eu(edge.vp[i], edge.vp[i + 1]);
    Posi p1 = edge.vp[i], p2 = edge.vp[i + 1];
    double x1 = (cur_p.x - p1.x) * ARG;
    double x2 = (p2.x - p1.x) * ARG;
    double y1 = (cur_p.y - p1.y) * ARG;
    double y2 = (p2.y - p1.y) * ARG;
    double bonus = x1 * x2 + y1 * y2;
    bonus = abs(bonus);
    bonus /= cqlt_pp_eu(edge.vp[i], edge.vp[i + 1]);
    dist += bonus;
    return dist;
}
double cqlt_p2p_onroad(const Posi &p1, const Posi &p2, const Road &edg1, const Road &edg2)
// 两轨迹点在地图上的距离
{
    double dist_road = 0, shl;
    double r1, r2;
    r1 = cqlt_start_to_posi(p1, edg1);
    r2 = cqlt_start_to_posi(p2, edg2);
    // if (edg1.id == edg2.id || edg2.id == friend_road[edg1.id])
    // {
    //     dist_road = abs(r1 - r2);
    // }
    if (edg1.id == edg2.id)
    {
        dist_road = abs(r1 - r2);
    }
    else
    {
        /*double len1=dis_Between_2point(*edg.unit.begin(),*(edg1.unit.end()-1));
        double len2=dis_Between_2point(*edg2.unit.begin(),*(edg2.unit.end()-1));*/
        double len1 = edg1.roadlen;

        if(shortestlens[edg1.to].find(edg2.from)==shortestlens[edg1.to].end())
            shl=STOPLEN*2;
        else
            shl=shortestlens[edg1.to][edg2.from];
        // dist_road = len1 - r1 + r2 + path_to_len(edg1, edg2);
        dist_road = len1+ r2 + shl - r1 ;
    }
    return abs(dist_road);
}

int fatheredge[MAX_NODE];
double distD[MAX_NODE];
// double dis_on_road[MAX_NODE][MAX_NODE];
void Dijkstra()
{
    //来自https://blog.csdn.net/tlonline/article/details/47398403
    for (int j = 0; j < MAX_NODE; j++)
        distD[j] = 5000.0;

    // memset(lastedge,0,sizeof(lastedge));
    for (int i = 0; i < MAX_NODE; i++)
    {
        map<int,double> shortestlen;
        map<int, int> path;
        map<int, int> changed;
        priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> q;

        distD[i] = 0;
        fatheredge[i] = 0;
        pair<double, int> p;
        q.push(make_pair(0, i));
        changed[i] = 1;
        // 记住哪些修改了
        while (!q.empty())
        {
            p = q.top(), q.pop();
            path[p.second] = fatheredge[p.second];
            for (int j = 0; j < head[p.second].size(); j++)
            {
                // cout<<dist[node[p.second][j].to]<<' ';
                if (distD[p.second] + edges[head[p.second][j]].roadlen < distD[edges[head[p.second][j]].to])
                {
                    distD[edges[head[p.second][j]].to] = distD[p.second] + edges[head[p.second][j]].roadlen;
                    fatheredge[edges[head[p.second][j]].to] = edges[head[p.second][j]].id;
                    q.push(make_pair(distD[edges[head[p.second][j]].to], edges[head[p.second][j]].to));
                    // cout<<distD[node[p.second][j].to]<<' ';
                    changed[edges[head[p.second][j]].to] = 1;
                }
            }
            if (q.top().first > STOPLEN || path.size() > 50)
                break;
        }

        for (auto it = changed.begin(); it != changed.end(); it++)
        {
            shortestlen[it->first]=distD[it->first];
            distD[it->first] = 5000.0;
            // lastedge[it->first]=0;
        }
        // 恢复
        // paths.emplace_back(path);
        shortestlens.emplace_back(shortestlen);
    }
}

// double path_to_len(const Road &eg_start, const Road &eg_end)
// {
//     int pre = eg_end.from;
//     int target = eg_start.to;
//     map<int, int> &path = paths[target];
//     // if (!path[eg_end.from])
//     //     return 0;
//     double ret = 0.0;
//     for (; path[pre] && pre != target; pre = edges[path[pre]].from)
//         ret += edges[path[pre]].roadlen;
//     return ret;
// }

// 来自https://github.com/RenchuSong/PRESS

double guancegailv(const Posi &p, const Road &r)
{
    double pownum, dist, ans;

    // double level = double(r.lev);
    // level = -level * 0.09 + 1;
    // level = 1 - 0.09 * (double)(r.lev);
    int i = 0;
    dist = cqlt_posi_to_edge(p, r);
    for (i = 0; i < r.vp.size() - 1; i++)
        if (no_dunjiao(p, r.vp[i], r.vp[i + 1]))
            break;

    if (i == r.vp.size() - 1)
        dist = dist * 5;
    // dist = level * dist * 1.2;
    dist*=1.2;
    pownum = -(dist / 0.00807) * (dist / 0.00807) / 2;
    // cout<<exp(pownum) / (0.05)<<' ';
    return exp(pownum) / (0.05);
}
const double Beta[31] = {0,
                         0.49037673,
                         0.82918373,
                         1.24364564,
                         1.67079581,
                         2.00719298,
                         2.42513007,
                         2.81248831,
                         3.15745473,
                         3.52645392,
                         4.09511775,
                         4.67319795,
                         5.41088180,
                         6.47666590,
                         6.29010734,
                         7.80752112,
                         8.09074504,
                         8.08550528,
                         9.09405065,
                         11.09090603,
                         11.87752824,
                         12.55107715,
                         15.82820829,
                         17.69496773,
                         18.07655652,
                         19.63438911,
                         25.40832185,
                         23.76001877,
                         28.43289797,
                         32.21683062,
                         34.56991141};

double zhuanyigailv(const Posi &p1, const Posi &p2, const Road &e1, const Road &e2, const int &t1, const int &t2)
{
    double dist;
    dist = cqlt_p2p_onroad(p1, p2, e1, e2);

    dist = abs(cqlt_pp_eu(p1, p2) - dist);
    if (t2 - t1 <= 30)
    {
        double m = 10.0 * dist / Beta[t2 - t1];
        // cout<<exp(-m)<<' ';
        return exp(-m) / Beta[t2 - t1];
    }
    else
    {
        double m = 10.0 * dist / 35.0;
        // cout<<exp(-m)<<' ';
        return exp(-m) / 35.0;
    }
}

void match(const Track &traj, vector<map<int, int>> &matchedEdgs, map<int, double> &curprog)
{
    //来自王宽宁同学的指导
    Posi_t pos_t;
    Posi cur_p, pre_p;
    int t1, t2;
    // vector<int> matched;
    int size_track = traj.track.size();
    for (int i = 0; i < size_track; i++)
    {
        pos_t = traj.track[i];
        cur_p.x = pos_t.x, cur_p.y = pos_t.y;
        t1 = pos_t.time;

        curprob.clear();
        preedge.clear();
        double alt = 0.0, temp_trans, temp_vis;
        int max_edge, second_edge;
        double max_prog, second_prog;

        pair<int, int> rowcol;
        rowcol = belong_to_grid(cur_p);
        vector<int> &gridEdgs = grids[rowcol.first][rowcol.second];

        // priority_queue<pair<double, int>> q;

        if (i && rowcol.first < 2000 && rowcol.second < 2000 && rowcol.first >= 0 && rowcol.second >= 0)
        {
            map<int, double> visprobs;
            for (int idx = 0; idx < gridEdgs.size(); idx++)
                visprobs[gridEdgs[idx]] = guancegailv(cur_p, edges[gridEdgs[idx]]);

            pre_p.x = traj.track[i - 1].x,
            pre_p.y = traj.track[i - 1].y;
            t2 = traj.track[i - 1].time;
            auto &preprog = curprog;

            for (int idx = 0; idx < gridEdgs.size(); idx++)
            {
                max_prog = second_prog = -100.0;
                second_edge = max_edge = -1;
                for (auto it = preprog.begin(); it != preprog.end(); it++)
                {
                    temp_trans = zhuanyigailv(pre_p, cur_p, edges[it->first], edges[gridEdgs[idx]], t2, t1);
                    // cout<<it->second<<' ';
                    alt = (it->second) * temp_trans;
                    // double standard=roadspeed[edges[gridEdgs[idx]].way_type];
                    // cout<<standard<<' ';
                    // sum=sum*(1-abs(standard-speed)/200.0);
                    // cout<<speed<<' ';
                    if (alt > max_prog)
                    {
                        second_prog = max_prog, second_edge = max_edge;
                        max_prog = alt, max_edge = it->first;
                    }
                    if (alt != max_prog && alt > second_prog)
                    {
                        second_prog = alt, second_edge = it->first;
                    }
                }
                // 优化，反向路
                // if (friend_road[max_edge])
                // {
                //     double x1, y1, x2, y2;
                //     int sz = edges[max_edge].vp.size();
                //     x1 = cur_p.x - pre_p.x, y1 = cur_p.y - pre_p.y;
                //     x2 = edges[max_edge].vp[sz - 1].x - edges[max_edge].vp[0].x;
                //     y1 = cur_p.y - pre_p.y;
                //     y2 = edges[max_edge].vp[sz - 1].y - edges[max_edge].vp[0].y;

                //     double judge = x1 * x2 + y1 * y2;
                //     if (judge < 0)
                //         max_edge = friend_road[max_edge];
                // }
                // // 优化，方向
                // else if (second_edge != -1)
                // {
                //     int presize = edges[max_edge].vp.size();
                //     int cursize = edges[second_edge].vp.size();
                //     double cos1 = (cur_p.x - pre_p.x) * (edges[max_edge].vp[presize - 1].x - edges[max_edge].vp[0].x) +
                //                   (cur_p.y - pre_p.y) * (edges[max_edge].vp[presize - 1].y - edges[max_edge].vp[0].y);
                //     double cos2 = (cur_p.x - pre_p.x) * (edges[second_edge].vp[cursize - 1].x - edges[second_edge].vp[0].x) +
                //                   (cur_p.y - pre_p.y) * (edges[second_edge].vp[cursize - 1].y - edges[second_edge].vp[0].y);
                //     double di = cqlt_pp_eu(cur_p, pre_p);
                //     if (cos1 < 0 && cos2 > 0 && di > 0.01)
                //     {
                //         max_edge = second_edge;
                //         max_prog = second_prog;
                //     }
                // }

                preedge[gridEdgs[idx]] = max_edge;
                curprob[gridEdgs[idx]] = visprobs[gridEdgs[idx]] * max_prog;
            }
            double maxprog = 0;
            // 归一化
            for (auto it_prob = curprob.begin(); it_prob != curprob.end(); it_prob++)
            {
                maxprog = max(maxprog, it_prob->second);
            }
            for (auto it_prob = curprob.begin(); it_prob != curprob.end(); it_prob++)
            {
                it_prob->second /= maxprog;
            }
            curprog = curprob;
        }
        else
        {
            map<int, double> visprobs;
            for (int idx = 0; idx < gridEdgs.size(); idx++)
            {
                visprobs[gridEdgs[idx]] = guancegailv(cur_p, edges[gridEdgs[idx]]);
            }
            double maxprog = 0;
            for (auto it_prob = visprobs.begin(); it_prob != visprobs.end(); it_prob++)
            {
                maxprog = max(maxprog, it_prob->second);
            }
            for (auto it_prob = visprobs.begin(); it_prob != visprobs.end(); it_prob++)
            {
                it_prob->second /= maxprog;
            }

            curprog = visprobs;
        }
        matchedEdgs.emplace_back(preedge);
        // hiddenProb.emplace_back(curprob);
    }
}

void print(vector<map<int, int>> &matchedEdgs, map<int, double> &progvector)
{

    double maxprob = -1.0;
    int maxedge;
    for (auto it = progvector.begin(); it != progvector.end(); it++)
    {
        if ((it->second) > maxprob)
        {
            maxprob = it->second;
            maxedge = it->first;
        }
        // cout<<it->second<<" ";
    }
    const int size_edge = matchedEdgs.size();
    int matched[size_edge];
    matched[size_edge - 1] = maxedge;
    for (int i = matchedEdgs.size() - 1; i > 0; i--)
    {
        maxedge = matchedEdgs[i][maxedge];
        // matched.emplace_back(edg);
        matched[i - 1] = maxedge;
    }

    // int i,j,k;
    // 减小误差
    for (int i = 0; i < size_edge; i++)
        for (int j = i + 2; j < size_edge&&j<i+6; j++)
            if (matched[i] == matched[j])
                for (int k = i + 1; k < j; k++)
                    matched[k] = matched[i];
                    

    for (int i = 0; i < size_edge; i++)
        printf("%d ", matched[i]);
    printf("\n");
}

int main()
{
    // freopen("sample.in", "r", stdin);
    // freopen("sample.out", "w", stdout);
    read();
    initial();
    Dijkstra();
    // cout << M << endl;
    printf("%d\n", M);
    for (int i = 0; i < tracks.size(); i++)
    {
        vector<map<int, int>> matchedEdgs;
        map<int, double> progvec;
        match(tracks[i], matchedEdgs, progvec);
        print(matchedEdgs, progvec);
    }
    return 0;
}
