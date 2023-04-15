

def Dijkstra(a, b, n):
    a -= 1
    b -= 1
    g = []
    for _ in range(n):
        g.append([])
    for i in range(n+20):
        g[m[i][0] - 1].append([m[i][1] - 1, m[i][2]])
    used = [False for _ in range(n)]
    dist = [10 ** 9 for _ in range(n)]
    dist[a] = 0
    prev = [-1 for _ in range(n)]
    for _ in range(n):
        mnv = -1
        mni = -1
        for j in range(n):
            if not used[j]:
                if mnv == -1:
                    mnv = dist[j]
                    mni = j
                else:
                    if dist[j] < mnv:
                        mnv = dist[j]
                        mni = j
        for j in range(len(g[mni])):
            if dist[g[mni][j][0]] > dist[mni] + g[mni][j][1]:
                dist[g[mni][j][0]] = dist[mni] + g[mni][j][1]
                prev[g[mni][j][0]] = mni
        used[mni] = True
    ans = []
    while prev[b] != -1:
        ans.append(b + 1)
        b = prev[b]
    ans.append(b + 1)
    ans.reverse()
    return ans