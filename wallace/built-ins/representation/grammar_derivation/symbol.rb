# encoding: utf-8
define_type :grammar_symbol do
  composer { s(:grammar_symbol) }
end

define_specification :grammar_symbol do

  # The value fo this literal.
  attribute :value, ruby(:string)

  # Constructs a new literal token.
  constructor [
    parameter(:value, ruby(:string))
  ] do
    self.value = value
  end

  method :literal?, returns: ruby(:boolean) do
    return false
  end

end
