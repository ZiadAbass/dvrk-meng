# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

# Ziad Additions
export EDITOR='nano -w'
alias edit='sudo nano ~/.bashrc'
alias editt='source ~/.bashrc'
alias c='clear'
alias ct='cd ~/catkin_ws'
alias srcc='cd ~/catkin_ws/src'
alias gits='git status'
alias py='python'
# Run the DVRK simulation with the default JSON
alias runsim='roslaunch dvrk_robot dvrk_arm_rviz.launch arm:=PSM1 config:=/home/ziad/catkin_ws/src/cisst-saw/sawIntuitiveResearchKit/share/console-PSM1_KIN_SIMULATED.json'
# Navigate to the virtualbox shared folder
alias dis='cd /media/sf_sharedvm/'
# navigate to the meng repo
alias meng='cd /media/sf_sharedvm/dvrk-meng/'
# run roscore and launch rviz for dvrk in background
# alias runall_old='roscore & (sleep 2; runsim)&'
# kill both roscore and dvrk rviz
alias killall='pkill roslaunch;pkill roscore'
# run the moveit API
alias move_api='roslaunch psm_moveit_config move_group.launch &'
# run the moveit GUI
alias move_gui='roslaunch psm_moveit_config demo.launch'
# run moveit demo without rviz (optimal)
alias move='roslaunch psm_moveit_config demo.launch use_rviz:=false'
# run roscore and launch rviz for dvrk in background
alias runall='roscore & (sleep 2; runsim)& (sleep 4; move)&'
# run roscore and launch rviz for dvrk in background AND use moveit's GUI
alias runallgui='roscore & (sleep 2; runsim)& (sleep 4; move_gui)&'
# reset the running nodes
alias rstall='killall & (sleep 3;runall)&'
# reset the running nodes with moveit's GUI
alias rstallgui='killall & (sleep 3;runallgui)&'

alias bye='sudo shutdown now'

source /opt/ros/melodic/setup.bash

if [ -f ~/catkin_ws/devel/setup.bash ]; then
  . ~/catkin_ws/devel/setup.bash
fi

# create a backup of the bashrc file inside the project repo
cp ~/.bashrc /media/sf_sharedvm/dvrk-meng/environment/bashrc_backup.sh


