from compas_fab.backends import PyBulletClient


with PyBulletClient(connection_type='direct') as client:
    print('Connected:', client.is_connected)