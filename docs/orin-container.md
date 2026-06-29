# Accessing the Orin Development Container


## 1. Connect to the Orin

Once the Orin is powered on, connect your laptop to it via Ethernet.

The Orin should be reachable over SSH at:

```bash
ssh moborobot@10.42.0.4
```

**Note:** I am not completely certain whether the last octet is `.4` or `.2`. If `.4` does not work, try `.2`.

For the quickest setup, configure your laptop's Ethernet interface with the static IP:

```text
10.42.0.1
```

There are other ways to establish connectivity, but using this address will likely save you time.


## 2. Verify the Development Folder

On the Orin, there should be a folder named:

```bash
devel
```

Inside this folder, there should be a hidden directory called:

```bash
.devcontainer
```

To view hidden files and directories, use:

```bash
cd devel && ls -la
```

The `.devcontainer` directory should have a layout similar to:

https://github.com/leenslf/duckie-coverage/tree/master/.devcontainer


## 3. Install Required VS Code Extensions on your host VS Code

Before proceeding, install the following VS Code extensions:

* Remote - SSH
* Docker
* Dev Containers

These extensions are required for the intended workflow.



## 4. Connect Through VS Code

Open VS Code and connect to the Orin using the Remote SSH extension.

A convenient way is:

1. Press `Ctrl+P`
2. Select **Remote-SSH: Connect to Host**
3. Enter the SSH configuration for the Orin
4. Connect

Once connected, open the project folder containing the `.devcontainer` directory.



## 5. Open the Development Container

When VS Code detects the `.devcontainer` configuration, it should prompt you to:

```text
Reopen in Container
```

Select **Reopen in Container**. If you didn't get prompted Ctrl+P and navigate to it. 

**Do not choose "Rebuild Container" unless you need to.** Rebuilding takes time. 

Once done reopening and setting the container up, you are now working inside the development container.



## 6. Accessing the Container Manually 

If you want to access the container from a terminal, you can.

First, list all containers:

```bash
docker ps -a
```

Locate the container ID (hash), then enter it with:

```bash
docker exec -it <container-id> bash
```

This only works if the container is running, if it isn't (you can't see it when you run `docker ps` alone), run:
```bash
docker start <container-id>
```
- `docker ps` → shows **running** containers only.
- `docker ps -a` → shows **all** containers, including stopped ones.
- `docker start <container>` → starts an existing stopped container.
- `docker exec -it <container> bash` → runs a shell inside a **running** container.
- `docker run <image>` → creates a **new container** from an image. It does not use a container 

That said, if you're already using VS Code inside the dev container, there is usually little reason to do this manually.
