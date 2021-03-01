

def exclude_closest_coord(target, options):
    diffs = []
    for option in options:
        if len(option) != len(target):
            print("Lengths in exclude_closest_coord don't match")
            return
        
        # get the average difference between all individual XYZcoordinates
        total_diff = 0
        for idx in range(0,len(target)):
            total_diff += abs(option[idx]-target[idx])
        avg_diff = float(total_diff/len(target))
        # print 'avg diff', avg_diff
        # keep track of all the average differences
        diffs.append(avg_diff)
    
    # find index of the smallest difference in distance
    si = diffs.index(min(diffs))
    del options[si]
    # return the same options minus the excluded coordinate
    return options
    

target = [17,17,17]
options = [[9,9,9],[12,12,12], [17,17,17], [13,13,13]]

res = exclude_closest_coord(target, options)

print res