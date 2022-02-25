list = []
string = 'ERROR'
with open('test_simple.log', 'r') as file:
  for line in file.readlines():
    if string in line:
        time = eval(line.split('|')[-1])
        list.append(time)
        
print(list[0])