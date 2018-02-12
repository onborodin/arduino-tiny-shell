

# Arduino tiny shell

Here I present sample arduino micro-application with
- ring buffer for input and output with hook to interrupts (wide shared ring buffer code with my adaptation and inmprovement)
- extensible tiny sheel for command execution
- some data manipulation function
- PWM timers and interrupts sample

Shell really is tiny and used only ~850 bytes of microcontroller code.

Well, if in truth, I wrote this code for the toys of my youngest son =)

We cann add own function call. The agreement about the called functions is very simple:
- function must accept string arguments `uint8_t*`
- and return an integer, now is `int16_t`


The syntax of the shell is also simple:

`command [arg [arg]];`

where `arg` can be string or signed/unsigned integer. There is nothing 
difficult to add your types. To do this, you need to write the translation
 functions that are called inside the command handler function.

The argument delimiter is a space, the command terminator is a semicolon `;`.


## Arduino screen

![](http://wiki.unix7.org/_media/c/screenshot-2018-02-12-09-58-27.png)












