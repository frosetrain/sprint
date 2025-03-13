f = open("out", "r+")
new_file = []
for line in f:
    better = " ".join(line.split()[0:-1])
    new_file.append(better)
with open("out", "w+") as f:
    for i in new_file:
        f.write(i + "\n")
