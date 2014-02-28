if [[ ! ":$PATH:" =~ (^|:)"`pwd`/gem5-orig/build/ARM"(:|$) ]]; then
    export PATH=$PATH:`pwd`/gem5-orig/build/ARM
fi
export GEM5=`pwd`/

if [[ ! ":$PATH:" =~ (^|:)"`pwd`/McPAT08release"(:|$) ]]; then
    export PATH=$PATH:`pwd`/McPAT08release
fi
