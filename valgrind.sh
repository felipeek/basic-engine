#!/bin/bash
valgrind --leak-check=full --dsymutil=yes --track-origins=yes --log-file=valgrind_output ./bin/basic-engine
