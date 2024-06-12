gcc -fPIC -shared -O3 hyumob_C.c -o hyumob_C.so -lm
gcc -fPIC -shared -O3 hyumob_G.c -o hyumob_G.so -lm
gcc -fPIC -shared -O3 hyumob_M.c -o hyumob_M.so -lm
gcc -fPIC -shared -O3 hyumob_Minv.c -o hyumob_Minv.so -lm
gcc -fPIC -shared -O3 hyumob_fd.c -o hyumob_fd.so -lm
gcc -fPIC -shared -O3 hyumob_fk.c -o hyumob_fk.so -lm
gcc -fPIC -shared -O3 hyumob_id.c -o hyumob_id.so -lm
gcc -fPIC -shared -O3 hyumob_J_s.c -o hyumob_J_s.so -lm
gcc -fPIC -shared -O3 hyumob_J_b.c -o hyumob_J_b.so -lm

mv ./*.so ../

