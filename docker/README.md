
In this repository, you will find the code and instructions to build the applications.

## About Docker

Docker is the default technology we will use to deploy, test, and develop software for this robot. It is available for Windows (with WSL), Mac, and Linux. There are many reasons why we are using Docker, and here’s a list of them:

- **Python has a lot of dependencies and versions.** The main programming language we will be developing in is Python. One common issue with Python is managing dependencies and libraries. This will definitely be the case for us as well. It’s easy to start with your default global environment and quickly run into issues because some libraries don’t match the expected versions, making it impossible to start developing. This is often called "Dependency Hell," and trust me, you’ll encounter it frequently if you don’t use Docker.

- **ROS versions.** This is essentially the same problem but for ROS. Surprisingly, ROS can cause even more issues, as there are two major ROS types, each with about five different distributions.

- **Deploying applications is hard.** Why? Because there are many manual configurations required for every deployment. Is it a website? Which port is it hosted on? Does the application need to be built, or are you using a developer mode? Are there issues with Cross-Origin Resource Sharing (CORS) for API requests? What are the hardware requirements? And why on earth does the database have 500,000 requests when there are only five users? Docker helps us mitigate these kinds of issues in robotics as well.

- **Managing different environment requirements.** Does the application only work if you have CUDA? How can we build the application without it? Are you running on ARM architecture but only have an AMD64 computer? Docker helps us solve these problems.

## Using Docker

To use Docker, you will first need to download it from their website: [https://www.docker.com/]. Make sure Docker is available in your terminal and that you can run the following commands:

```bash
docker
docker-compose
```

*TODO: Continue writing this README.*


# WRITE about how to use QEMU in docker


# How to develop without having a GPU with CUDA?


