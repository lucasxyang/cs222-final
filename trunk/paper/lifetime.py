batt = 2122 * 60 # maS
drain = 3.4 # ma
packetsPerSecond = (100 * 12 * 3) / 8.0 / 30.0
costPerPacket = .006 * 19.6 # maS

ratios = [1, .95, .9, .85, .8, .75, .7, .65, .6, .55, .5, .45, .4, .35, .30, .25, .2]

baseline = 6.8487037037

for r in ratios:
    t = 0
    e = batt
    timePerPacket = 1.0/(packetsPerSecond*r)
    while e > 0:
        e -= timePerPacket*drain
        e -= costPerPacket
        t += timePerPacket
    print r, t/3600.0, ((t/3600.0)/baseline -1) * 100
