---
title: OpenSSF专题六：ttl.sh使用
date: 2022-01-18 15:07:16
tags:
---

# 一个简单的Dockfile

一、Dockerfile 概念
Docker 镜像是一个特殊的文件系统，除了提供容器运行时所需的程序、库、资源、配置等文件外，还包含了一些为运行时准备的一些配置参数（如匿名卷、环境变量、用户等）。镜像不包含任何动态数据，其内容在构建之后也不会被改变。

镜像的定制实际上就是定制每一层所添加的配置、文件。如果我们可以把每一层修改、安装、构建、操作的命令都写入一个脚本，用这个脚本来构建、定制镜像，那么之前提及的无法重复的问题、镜像构建透明性的问题、体积的问题就都会解决。这个脚本就是 Dockerfile。

Dockerfile 是一个文本文件，其内包含了一条条的指令(Instruction)，每一条指令构建一层，因此每一条指令的内容，就是描述该层应当如何构建。有了 Dockerfile，当我们需要定制自己额外的需求时，只需在 Dockerfile 上添加或者修改指令，重新生成 image 即可，省去了敲命令的麻烦。
二、Dockerfile文件格式

格式如下：

```
##  Dockerfile文件格式

# This dockerfile uses the ubuntu image
# VERSION 2 - EDITION 1
# Author: docker_user
# Command format: Instruction [arguments / command] ..
 
# 1、第一行必须指定 基础镜像信息
FROM ubuntu
 
# 2、维护者信息
MAINTAINER docker_user docker_user@email.com
 
# 3、镜像操作指令
RUN echo "deb http://archive.ubuntu.com/ubuntu/ raring main universe" >> /etc/apt/sources.list
RUN apt-get update && apt-get install -y nginx
RUN echo "\ndaemon off;" >> /etc/nginx/nginx.conf
```

Dockerfile 分为四部分：基础镜像信息、维护者信息、镜像操作指令、容器启动执行指令。一开始必须要指明所基于的镜像名称，接下来一般会说明维护者信息；后面则是镜像操作指令，例如 RUN 指令。每执行一条RUN 指令，镜像添加新的一层，并提交；最后是 CMD 指令，来指明运行容器时的操作命令。

三、构建镜像

docker build 命令会根据 Dockerfile 文件及上下文构建新 Docker 镜像。构建上下文是指 Dockerfile 所在的本地路径或一个URL（Git仓库地址）。构建上下文环境会被递归处理，所以构建所指定的路径还包括了子目录，而URL还包括了其中指定的子模块。

将当前目录做为构建上下文时，可以像下面这样使用docker build命令构建镜像：

```
docker build .
Sending build context to Docker daemon  6.51 MB
...
```

说明：构建会在 Docker 后台守护进程（daemon）中执行，而不是CLI中。构建前，构建进程会将全部内容（递归）发送到守护进程。大多情况下，应该将一个空目录作为构建上下文环境，并将 Dockerfile 文件放在该目录下。

在构建上下文中使用的 Dockerfile 文件，是一个构建指令文件。为了提高构建性能，可以通过.dockerignore文件排除上下文目录下不需要的文件和目录。

在 Docker 构建镜像的第一步，docker CLI 会先在上下文目录中寻找.dockerignore文件，根据.dockerignore 文件排除上下文目录中的部分文件和目录，然后把剩下的文件和目录传递给 Docker 服务。

Dockerfile 一般位于构建上下文的根目录下，也可以通过-f指定该文件的位置：

```
docker build -f /path/to/a/Dockerfile .
```



# ttl.sh使用

[ttl.sh](https://ttl.sh/) 是一个域名，它提供了免费、短期（以小时计）的、匿名的容器镜像托管。具体操作如下：

```
$ IMAGE_NAME=$(uuidgen)			# 以UUID作为测试镜像的名称
$ docker build -t ttl.sh/${IMAGE_NAME}:1h .  # 需要在当前路径下存在一个Dockerfile文件，执行后就会创建名为ttl.sh/${IMAGE_NAME}的镜像
$ docker push ttl.sh/${IMAGE_NAME}:1h
 
 ................................................
 image ttl.sh/xxxx-yyyy-nnnn-2a2222-4b44 is available for 1 hour
 
 ttl.sh is contributed by Replicated (www.replicated.com)
```



![image-20220118150801726](/home/ubuntu-ros2/myBlog/source/_posts/OpenSSF专题六：ttl-sh使用/image-20220118150801726.png)

![image-20220118153625677](/home/ubuntu-ros2/myBlog/source/_posts/OpenSSF专题六：ttl-sh使用/image-20220118153625677.png)

![image-20220118153750320](/home/ubuntu-ros2/myBlog/source/_posts/OpenSSF专题六：ttl-sh使用/image-20220118153750320.png)

![image-20220118154351562](/home/ubuntu-ros2/myBlog/source/_posts/OpenSSF专题六：ttl-sh使用/image-20220118154351562.png)
