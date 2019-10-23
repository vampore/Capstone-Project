s = serial('COM4','BAUD',9600);

fopen(s);

for i = 1:5
    value = input("Enter the value 100 to turn on and 101 to turn off");
    
    fprintf(s,value);
    
end