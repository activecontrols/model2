import time

Import("env")

def after_upload(source, target, env):
    print("Waiting 3 seconds for portenta to restart...")
    time.sleep(3)

env.AddPostAction("upload", after_upload)