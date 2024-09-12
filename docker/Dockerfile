FROM julia:1.10.4-bookworm

# System updates and installation of necessary packages
RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -y git sudo xvfb libgl1-mesa-dri libgl1-mesa-glx libglu1-mesa texlive-full
RUN apt-get install -y python3 python3-pip python3-dev
RUN apt-get install -y python3-matplotlib
RUN apt-get install -y git
RUN apt-get install -y zsh
RUN curl -fsSL https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh | sh
RUN apt-get clean
RUN rm -rf /var/lib/apt/lists/*

# Set up Python environment
ENV PYTHON=python3

RUN mkdir /project/

RUN cd /tmp \
    && git clone https://github.com/UW-ACL/SCPToolbox.jl.git \
    && cd SCPToolbox.jl \
    && git checkout master \
    && julia --project -e 'using Pkg; Pkg.instantiate(); Pkg.build("PyCall"); Pkg.precompile()'

# Install Julia packages
RUN julia -e 'using Pkg; Pkg.add(["JuMP", "Plots", "Optim", "Ipopt", "GR", "PyCall", "PyPlot", "Colors", "ECOS"])'

ENV DISPLAY=:0
ENV GKSwstype=x11

RUN cd \
    && julia --project=. -e 'using Pkg; Pkg.add(url="https://github.com/UW-ACL/SCPToolbox.jl")'