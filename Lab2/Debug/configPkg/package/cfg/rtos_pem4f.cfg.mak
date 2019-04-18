# invoke SourceDir generated makefile for rtos.pem4f
rtos.pem4f: .libraries,rtos.pem4f
.libraries,rtos.pem4f: package/cfg/rtos_pem4f.xdl
	$(MAKE) -f C:\Users\doate\Desktop\ECE-3849\ECE-3849-Labs\Lab2/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\doate\Desktop\ECE-3849\ECE-3849-Labs\Lab2/src/makefile.libs clean

