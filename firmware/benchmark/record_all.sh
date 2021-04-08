for i in 82b57efa1 88f58aba4 242d2b9bd 725a3f8f1 f68a41072; do git reset --hard && git checkout $i && bash collect.sh "$i" || echo fail; done
