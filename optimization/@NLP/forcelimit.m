function nlp = forcelimit(nlp, rbm, Con, Fcx,Fcy)
        
nlp = AddConstraint(nlp, Fcx, 0, 0.005 ,'friction');

end
