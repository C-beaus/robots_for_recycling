import matplotlib.pyplot as plt
from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

# log_dir = "runs/mobilenet_ss_18_wd_0001_class_dataset/validation"
log_dir = "runs/resnet_ss_18_wd_0001_class_dataset/validation"
event_acc = EventAccumulator(log_dir)
event_acc.Reload()

for tag in event_acc.Tags()['scalars']:
    steps = [scalar.step for scalar in event_acc.Scalars(tag)]
    values = [scalar.value for scalar in event_acc.Scalars(tag)]
    
    plt.plot(steps, values, label=tag)

plt.xlabel("Epoch")
plt.ylabel("Accuracy")
plt.legend()
plt.ylim(0,100)
plt.grid(True)
plt.minorticks_on()
plt.show()