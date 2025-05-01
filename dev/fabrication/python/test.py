from compas_fab.backends import RosClient

ros = RosClient('host.docker.internal', 9091)
ros.run(5)
print("Connected:", ros.is_connected)