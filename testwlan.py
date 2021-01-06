import os
import subprocess
import time


pilan = input("what is the lan? ")
out = open("pingresponse.txt", "a")

subprocess.call(["ping", pilan], stdout=out)

out.close()

