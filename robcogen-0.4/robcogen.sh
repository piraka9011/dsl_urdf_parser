#!/bin/bash

java -Dlog4j.configuration=file:log4j.properties -jar robcogen.jar $1 $2
