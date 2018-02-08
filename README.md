

# Arduino tiny shell

Here I present sample arduino micro-application with
- ring buffer for input and output with hook to interrupts (wide shared ring buffer code with my adaptation and inmprovement)
- expanded tiny sheel for command execution.
- some data manipulation function 

Shell really is tiny and used only ~850 bytes of microcontroller code.

We cann add own function call. The agreement about the called functions is very simple:
- function must accept string arguments `uint8_t*`
- and return an integer, now is `int16_t`


The syntax of the shell is also simple:

`command [arg [arg]];`

where `arg` can be string or signed/unsigned integer. There is nothing 
difficult to add your types. To do this, you need to write the translation
 functions that are called inside the command handler function.

The argument delimiter is a space, the command terminator is a semicolon `;`.












