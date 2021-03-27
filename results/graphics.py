import re
import matplotlib.pyplot as plt
import numpy as np

def autolabel(rects, axss):
    """Attach a text label above each bar in *rects*, displaying its height."""
    for rect in rects:
        height = rect.get_height()
        axss.annotate('{}'.format(height),
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(0, 3),  # 3 points vertical offset
                    textcoords="offset points",
                    ha='center', va='bottom')
                    
hand = open('/home/robot/github/HeMuRo-Framework/logs/Logger21/Missions.txt')

mission_owner = dict()
mission_executioner = dict()
mission_redirected = dict()

for line in hand:
    line = line.rstrip()
    
    missions = re.findall('.*null$', line)
    for mission in missions:
        words = line.split(', ')
        if len(words) == 7:
            owner = words[1]
            
            mission_owner[owner] = mission_owner.get(owner, 0 ) + 1

    missions = re.findall('.*Complete$', line)
    for mission in missions:
        words = line.split(', ')
        if len(words) == 7:
            executioner = words[2]
            deadline = int(words[3])
            estimated = int(words[4])
            realtime = int(words[5])
            
            mission_executioner[executioner] = mission_executioner.get(executioner, 0) + 1

    missions = re.findall('\[.*\]\[(.*)\].*Redirected$', line)
    for mission in missions:
        words = line.split(', ')
        if len(words) == 7 and words[2] == mission:
            executioner = words[2]
            
            mission_redirected[executioner] = mission_redirected.get(executioner, 0 ) + 1

print("Mission Offered:")
print(mission_owner)
print("Mission Completed")
print(mission_executioner)
print("Mission Redirected")
print(mission_redirected)


label_onwer = list(mission_owner.keys())
size_onwer = list(mission_owner.values())
explode_owner = [0] * len(size_onwer)

for agent in list(mission_redirected.keys()):
    mission_executioner[agent] = mission_executioner.get(agent, 0 )
for agent in list(mission_executioner.keys()):
    mission_redirected[agent] = mission_redirected.get(agent, 0 )

label_executioner = list(mission_executioner.keys())
size_executioner = list(mission_executioner.values())
explode_executioner = [0] * len(size_executioner)

label_redirected = list(mission_executioner.keys())
ordered_redirected = list()
for agent in list(mission_redirected.keys()):
	ordered_redirected.insert(-1,mission_redirected.get(agent))
print(ordered_redirected)
explode_redirected = [0] * len(ordered_redirected)

fig, axs = plt.subplots(1, 3, figsize=(15, 3))

rects0 = axs[0].bar(label_onwer, size_onwer)
axs[0].axis('equal')  # Equal aspect ratio ensures that pie is drawn as a circle.
axs[0].set(title= "Mission Owners")
axs[0].set_ylabel('Missions')

x = np.arange(len(label_redirected))  # the label locations
width = 0.05  # the width of the bars

rects1 = axs[1].bar(x - width/2, size_executioner, width, label='Completed')
rects2 = axs[1].bar(x + width/2, ordered_redirected, width, label='Redirected')

# Add some text for labels, title and custom x-axis tick labels, etc.
axs[1].set_ylabel('Missions')
axs[1].set_title('Missions Completed and Redirected')
axs[1].set_xticks(x)
axs[1].set_xticklabels(label_executioner)
axs[1].legend()


autolabel(rects0, axs[0])
autolabel(rects1, axs[1])
autolabel(rects2, axs[1])

fig.tight_layout()

axs[2].pie(size_executioner, explode=explode_executioner, labels=label_executioner, autopct='%1.1f%%', shadow=True, startangle=90)
axs[2].axis('equal')  # Equal aspect ratio ensures that pie is drawn as a circle.
axs[2].set(title= "Mission Executed")

#axs[2].pie(size_redirected, explode=explode_redirected, labels=label_redirected, autopct='%1.1f%%', shadow=True, startangle=90)
#axs[2].axis('equal')  # Equal aspect ratio ensures that pie is drawn as a circle.
#axs[2].set(title= "Mission Redirected")

plt.show()
