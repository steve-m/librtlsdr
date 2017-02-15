[![librtlsdr version](https://img.shields.io/github/tag/librtlsdr/librtlsdr.svg?style=flat&label=librtlsdr)](https://github.com/librtlsdr/librtlsdr/releases)
[![Build Status](http://circleci-badges-max.herokuapp.com/img/librtlsdr/librtlsdr/master?token=:circle-ci-token)](https://circleci.com/gh/librtlsdr/librtlsdr/tree/master)
[![GPLv2 License](http://img.shields.io/badge/license-GPLv2-brightgreen.svg)](https://tldrlegal.com/license/gnu-general-public-license-v2)

# Description

rtl-sdr turns your Realtek RTL2832 based DVB dongle into a SDR receiver


# For more information see:

http://sdr.osmocom.org/trac/wiki/rtl-sdr


# Contributing

Pull requests are always welcome but please make changes to, and pull request from, the development branch.

Initial setup:

- fork the main librtlsdr repo via github
- clone your fork locally and cd to the cloned repo's folder
- add the upstream development repo:
    * git remote add upstream git@github.com:librtlsdr/librtlsdr.git
- track the development branch: 
    * git branch --track development origin/development

Normal workflow:

- checkout the development branch and make your changes
- commit your changes
- sync your local development branch with the upstream development branch:
    * git fetch upstream
    * git merge upstream/development
- push your commit/s to your forked repo
- do a pull request via github
