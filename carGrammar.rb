def grammar
return {


'main' => ['

    private void insideJunction() {
        <insideJunction>
    }

    private void insideJunctionApproach() {

    }

    private void doingUTurnFromJunction() {

    }

    private void gettingCloseToObject() {
        <gettingClose>
    }

    private void isCloseToObject() {
 
    }

    private boolean shouldWeAdjust() {
        return false;

    }

    private void notInJunction() {
        <notInJunction>
    }

}


'],


# Literal information.
'nz_digit' => ['1', '2', '3', '4', '5', '6', '7', '8', '9'],
'digit' => ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9'],
'fractional' => ['<digit>', '<digit><digit>'],
'literal_boolean' => ['true', 'false'],
'literal_int' => ['<nz_digit><literal_int>', '<digit>'],
'literal_double' => ['<literal_int>.<fractional>'],

# Returns an integer.
'int' => [
'<literal_int>',

'Math.round(<float>)'
],

# Returns a floating point.
'double' => [
'<literal_double>',
'(double) <int>',
'findImminentCrash(sim.cars)',
'findImminentCrash(sim.ugvs)',
'sim.environment.getObjectLocation(sim.ugvs.top()).getX()',
'sim.environment.getObjectLocation(sim.ugvs.top()).getY()',
'sim.environment.getObjectLocation(this).getY()',
'sim.environment.getObjectLocation(this).getY()',
'getSpeed()'
],

# Returns a numeric type.
'numeric' => ['<double>', '<int>'],

# Returns a boolean.
'expression_boolean' => [
'<expression_boolean_term>'
],
'expression_boolean_term' => [
'<numeric> == <numeric>',
'<numeric> != <numeric>',
'<numeric> > <numeric>',
'<numeric> >= <numeric>',
'<numeric> < <numeric>',
'<numeric> <= <numeric>'
],
'bool' => [
'<literal_boolean>',
'<expression_boolean>'
],

# Composes a series of statements.
'statements' => [
'<statement> <statements>',
'<statement>'
],
'statement' => [
' if (<bool>) {
<statements>
}
else {
    <statements>
}',
'<action>;'
],
# Robocode tank actions.
'action' => [
'setTargetID(<int>)',
'goSlow()',
'goReallySlow()',
'goSlowStop()',
'goFaster(<bool>)',
'turnLeft(<double>)',
'turnRight(<double>)',
'emergencyStop()',
],
# The body of the while loop in the "run" method.

'insideJunction'=>['<statements>'],
'gettingClose'=>['<statements>'],
'notInJunction'=>['<statements>']
#'shouldWeAdjust'=>['<bool>']

}


end
